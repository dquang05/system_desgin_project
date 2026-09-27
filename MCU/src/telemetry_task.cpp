/**
 * @file telemetry_task.cpp
 * @brief FreeRTOS task for sending telemetry data over UDP.
 *
 * Packages the robot's current state (RPM, PWM, encoder values, ADC, loadcell)
 * into a JSON string and sends it via Wi-Fi to a ground station for logging.
 */
#include "../include/telemetry_task.hpp"
#include "../include/main.hpp"
#include "../include/shared_state.hpp"
#include "../lib/wifi_manager/wifi_manager.hpp"
#include <cstdio>
#include <esp_log.h>
#include <esp_timer.h>
#include <driver/gpio.h>

extern wifi_manager::WifiManager wifi;
static const char *TAG_IDLE = "TELE_IDLE";

/**
 * @brief Main telemetry task routine (Legacy continuous logging).
 *
 * Runs at a 20Hz frequency. Takes an atomic snapshot of the shared state,
 * serializes it to a JSON string, and sends it over UDP.
 *
 * @param pvParameters Pointer to the global SharedRobotState.
 */
void telemetry_task_routine(void *pvParameters) {
  SharedRobotState *state = static_cast<SharedRobotState *>(pvParameters);
  const TickType_t freq_ticks = pdMS_TO_TICKS(20); // 50Hz Logging Rate
  TickType_t last_wake_time = xTaskGetTickCount();
  char json_buf[1024];

  while (true) {
    // Read isolated snapshot
    SharedRobotState local_state;
    portENTER_CRITICAL(&state->spinlock);
    local_state = *state;
    portEXIT_CRITICAL(&state->spinlock);

    // Serialize data
    // Optimization: Use integer literal (1000) for division instead of 1000ULL
    int len = snprintf(
        json_buf, sizeof(json_buf),
        "{\"ts\":%lu,\"enc\":[%lld,%lld],\"pwm\":[%.2f,%.2f],\"adc\":[%lu,%lu,%"
        "lu,%lu,%lu],\"rpm_tgt\":[%.2f,%.2f],\"rpm_act\":[%.2f,%.2f],\"e2\":%."
        "2f,\"weight\":%.2f,\"v_ref\":%.2f,\"v_ref_turn\":%.2f,\"pid\":{\"L\":["
        "%.4f,%.4f,%.4f],"
        "\"R\":[%.4f,%.4f,%.4f],\"T\":[%.4f,%.4f,%.4f],\"T1\":[%.4f,%.4f,%.4f],"
        "\"T2\":[%.4f,%.4f,%.4f],\"W\":[%.3f,%.3f]},"
        "\"fuzzy\":%d,\"blind\":{\"s1\":[%.2f,%.2f,%lld,%lld],\"s2\":[%.2f,%.2f,%lld,%lld],\"s3\":[%.2f,%.2f]}}",
        (uint32_t)(esp_timer_get_time() / 1000), local_state.encoder_left,
        local_state.encoder_right, local_state.pwm_left, local_state.pwm_right,
        local_state.adc_raw[0], local_state.adc_raw[1], local_state.adc_raw[2],
        local_state.adc_raw[3], local_state.adc_raw[4],
        local_state.target_rpm_left, local_state.target_rpm_right,
        local_state.actual_rpm_left, local_state.actual_rpm_right,
        local_state.current_e2, local_state.loadcell_weight,
        local_state.physical_config.v_ref,
        local_state.physical_config.v_ref_turn,
        local_state.physical_config.kp_l, local_state.physical_config.ki_l,
        local_state.physical_config.kd_l, local_state.physical_config.kp_r,
        local_state.physical_config.ki_r, local_state.physical_config.kd_r,
        local_state.physical_config.kp,
        local_state.physical_config.kd,
        local_state.physical_config.pid_tau,
        local_state.physical_config.kp_load1,
        local_state.physical_config.kd_load1,
        local_state.physical_config.pid_tau_load1,
        local_state.physical_config.kp_load2,
        local_state.physical_config.kd_load2,
        local_state.physical_config.pid_tau_load2,
        local_state.physical_config.sensor_weight_04,
        local_state.physical_config.sensor_weight_13,
        local_state.track_config.fuzzy_mode ? 1 : 0,
        local_state.track_config.blind_seg1_rpm_l,
        local_state.track_config.blind_seg1_rpm_r,
        local_state.track_config.blind_seg1_pulses_l,
        local_state.track_config.blind_seg1_pulses_r,
        local_state.track_config.blind_seg2_rpm_l,
        local_state.track_config.blind_seg2_rpm_r,
        local_state.track_config.blind_seg2_pulses_l,
        local_state.track_config.blind_seg2_pulses_r,
        local_state.track_config.blind_seg3_rpm_l,
        local_state.track_config.blind_seg3_rpm_r);

    // Decoupled hardware transmission
    if (len > 0 && wifi.is_connected()) {
      wifi.send_log_data(UDP_TARGET_IP, UDP_TARGET_PORT,
                         reinterpret_cast<const uint8_t *>(json_buf), len);
    }

    vTaskDelayUntil(&last_wake_time, freq_ticks);
  }
}

/**
 * @brief Power-saving idle telemetry and Wi-Fi manager task.
 *
 * Runs at a 2Hz frequency (every 500ms).
 * - Monitors robot_state.system_running.
 * - When robot is RUNNING: Automatically stops Wi-Fi (wifi.stop()) to eliminate
 *   RF power amplifier current spikes and voltage sags.
 * - When robot is IDLE: Automatically starts Wi-Fi (wifi.start()). When connected,
 *   transmits a slim ~150B JSON packet with PID, v_ref, v_ref_turn, and sensor weights
 *   to serve as a heartbeat and confirmation for UI parameter tuning.
 *
 * @param pvParameters Pointer to the global SharedRobotState.
 */
void telemetry_idle_task_routine(void *pvParameters) {
  SharedRobotState *state = static_cast<SharedRobotState *>(pvParameters);
  const TickType_t freq_ticks = pdMS_TO_TICKS(500); // 2Hz
  TickType_t last_wake_time = xTaskGetTickCount();
  char json_buf[384];

  // Configure physical Wi-Fi switch pin (GPIO 4) as input with pull-up
  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = (1ULL << PIN_WIFI_SWITCH);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&io_conf);

  bool wifi_active = true; // Wi-Fi is initialized at boot in app_main

  while (true) {
    bool is_running = false;
    portENTER_CRITICAL(&state->spinlock);
    is_running = state->system_running;
    portEXIT_CRITICAL(&state->spinlock);

    // Hardware switch check (Active LOW: 0 = Allowed/ON, 1 = Forced OFF)
    int switch_level = gpio_get_level(PIN_WIFI_SWITCH);
    bool hw_switch_allowed = (switch_level == 0);

    // Desired Wi-Fi state: Only ON when robot is IDLE and hardware switch is ON
    bool desired_wifi = hw_switch_allowed && (!is_running);

    if (desired_wifi) {
      if (!wifi_active) {
        ESP_LOGI(TAG_IDLE, "Robot IDLE: Starting Wi-Fi...");
        wifi.start();
        wifi_active = true;
      }

      // Transmit slim heartbeat telemetry if connected
      if (wifi.is_connected()) {
        RobotPhysicalConfig phys_cfg;
        portENTER_CRITICAL(&state->spinlock);
        phys_cfg = state->physical_config;
        portEXIT_CRITICAL(&state->spinlock);

        int len = snprintf(
            json_buf, sizeof(json_buf),
            "{\"ts\":%lu,\"v_ref\":%.2f,\"v_ref_turn\":%.2f,\"pid\":{\"L\":["
            "%.4f,%.4f,%.4f],"
            "\"R\":[%.4f,%.4f,%.4f],\"T\":[%.4f,%.4f,%.4f],\"T1\":[%.4f,%.4f,%.4f],"
            "\"T2\":[%.4f,%.4f,%.4f],\"W\":[%.3f,%.3f]}}",
            (uint32_t)(esp_timer_get_time() / 1000),
            phys_cfg.v_ref,
            phys_cfg.v_ref_turn,
            phys_cfg.kp_l, phys_cfg.ki_l, phys_cfg.kd_l,
            phys_cfg.kp_r, phys_cfg.ki_r, phys_cfg.kd_r,
            phys_cfg.kp, phys_cfg.kd, phys_cfg.pid_tau,
            phys_cfg.kp_load1, phys_cfg.kd_load1, phys_cfg.pid_tau_load1,
            phys_cfg.kp_load2, phys_cfg.kd_load2, phys_cfg.pid_tau_load2,
            phys_cfg.sensor_weight_04, phys_cfg.sensor_weight_13);

        if (len > 0) {
          wifi.send_log_data(UDP_TARGET_IP, UDP_TARGET_PORT,
                             reinterpret_cast<const uint8_t *>(json_buf), len);
        }
      }
    } else {
      // Robot is RUNNING or hardware switch is turned OFF
      if (wifi_active) {
        ESP_LOGI(TAG_IDLE, "Robot RUNNING or Switch OFF: Stopping Wi-Fi...");
        wifi.stop();
        wifi_active = false;
      }
    }

    vTaskDelayUntil(&last_wake_time, freq_ticks);
  }
}
