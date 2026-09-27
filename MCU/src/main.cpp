/**
 * @file main.cpp
 * @brief Main entry point and orchestrator for the ESP32 AMR firmware.
 *
 * This file contains the FreeRTOS task definitions, global state management,
 * and hardware initialization. It acts as the central coordinator for all
 * decoupled library modules.
 */
#include <cJSON.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lwip/sockets.h>
#include <nvs.h>
#include <nvs_flash.h>

#include "../include/main.hpp"
#include "../include/motion_task.hpp"
#include "../include/shared_state.hpp"
#include "../include/telemetry_task.hpp"

#include "../lib/adc_dma/adc_dma.hpp"
#include "../lib/loadcell_hx711/loadcell_hx711.hpp"
#include "../lib/tb6612_encoder/tb6612_encoder.hpp"
#include "../lib/velocity_pid/velocity_pid.hpp"
#include "../lib/wifi_manager/wifi_manager.hpp"

static const char *TAG = "ORCHESTRATOR";

/**
 * ============================================================================
 * \todo 3-SEGMENT GAIN SCHEDULING & PID PROFILE PLAN
 * ============================================================================
 * \par Goal:
 * Change 3 package of PID parameter for diferrents road segments

 * ============================================================================
 */

// Global Shared State
SharedRobotState robot_state = {
    .spinlock = portMUX_INITIALIZER_UNLOCKED,
    .adc_raw = {0},
    .encoder_left = 0,
    .encoder_right = 0,
    .pwm_left = 0.0f,
    .pwm_right = 0.0f,
    .target_rpm_left = 0.0f,
    .target_rpm_right = 0.0f,
    .actual_rpm_left = 0.0f,
    .actual_rpm_right = 0.0f,
    .manual_cmd_l = 0.0f,
    .manual_cmd_r = 0.0f,
    .current_e2 = 0.0f,
    .loadcell_weight = 0.0f,
    .line_calib = {.x_max = {4095, 4095, 4095, 4095, 4095},
                   .x_min = {5, 0, 0, 23, 21},
                   .y_max = 1000,
                   .y_min = 0,
                   .line_coe_1 = 1.229f,  // 1.0
                   .line_coe_2 = 1.272f}, // 0/0
    .physical_config = {.wheel_base_mm = DEFAULT_PHYS_WHEEL_BASE_MM,
                        .wheel_radius_mm = DEFAULT_PHYS_WHEEL_RADIUS_MM,
                        .sensor_distance_mm = DEFAULT_PHYS_SENSOR_DISTANCE_MM,
                        .v_ref = 200.0f, // 200 mm/s base forward velocity
                        .v_ref_turn = 500.0f,
                        .kp = DEFAULT_KP,
                        .kd = DEFAULT_KD,
                        .pid_tau = DEFAULT_PID_TAU,
                        .kp_load1 = DEFAULT_KP_LOAD1,
                        .kd_load1 = DEFAULT_KD_LOAD1,
                        .pid_tau_load1 = DEFAULT_PID_TAU_LOAD1,
                        .kp_load2 = DEFAULT_KP_LOAD2,
                        .kd_load2 = DEFAULT_KD_LOAD2,
                        .pid_tau_load2 = DEFAULT_PID_TAU_LOAD2,
                        .sensor_weight_04 = DEFAULT_SENSOR_WEIGHT_04,
                        .sensor_weight_13 = DEFAULT_SENSOR_WEIGHT_13,
                        .kp_l = DEFAULT_KP_L,
                        .ki_l = DEFAULT_KI_L,
                        .kd_l = DEFAULT_KD_L,
                        .kp_r = DEFAULT_KP_R,
                        .ki_r = DEFAULT_KI_R,
                        .kd_r = DEFAULT_KD_R},
    .track_config = {.encoder_ppr = 341.2f,
                     .turn_phase1_outer_rpm = 50.0f,
                     .turn_phase1_inner_rpm = 0.0f,
                     .turn_phase1_timeout_ticks = 8, // 6 ticks * 50ms = 400ms
                     .loadcell_type1_min = 800.0f,
                     .loadcell_type1_max = 1200.0f,
                     .loadcell_type2_min = 1800.0f,
                     .loadcell_type2_max = 2200.0f,
                     .fuzzy_mode = DEFAULT_FUZZY_MODE,
                     .blind_seg1_rpm_l = DEFAULT_BLIND_SEG1_RPM_L,
                     .blind_seg1_rpm_r = DEFAULT_BLIND_SEG1_RPM_R,
                     .blind_seg1_pulses_l = DEFAULT_BLIND_SEG1_PULSES_L,
                     .blind_seg1_pulses_r = DEFAULT_BLIND_SEG1_PULSES_R,
                     .blind_seg2_rpm_l = DEFAULT_BLIND_SEG2_RPM_L,
                     .blind_seg2_rpm_r = DEFAULT_BLIND_SEG2_RPM_R,
                     .blind_seg2_pulses_l = DEFAULT_BLIND_SEG2_PULSES_L,
                     .blind_seg2_pulses_r = DEFAULT_BLIND_SEG2_PULSES_R,
                     .blind_seg3_rpm_l = DEFAULT_BLIND_SEG3_RPM_L,
                     .blind_seg3_rpm_r = DEFAULT_BLIND_SEG3_RPM_R,
                     .pickup_stop_pulses = 10800,
                     .pickup_decel_pulses = 10000,
                     .finish_stop_pulses = 10000,
                     .finish_decel_pulses = 10000,
                     .decel_ratio = 0.65f,
                     .finish_decel_ratio = 0.40f},
    .system_running = false,
    .soft_stop_request = false,
    .test_mode_active = false,
    .test_target_rpm_l = 0.0f,
    .test_target_rpm_r = 0.0f};

// Global Drivers (Workers)
wifi_manager::WifiManager wifi;
EspAdcDmaDriver adc_driver;
Tb6612Encoder motor_left;
Tb6612Encoder motor_right;
VelocityPid pid_left;
VelocityPid pid_right;
LoadcellHX711 loadcell(PIN_LOADCELL_DT, PIN_LOADCELL_SCK);

/**
 * @brief ADC DMA Polling Task.
 *
 * Continuously polls the ADC DMA buffer and updates the global state safely.
 * This runs on Core 1 with very high priority (6) to prevent buffer overflows.
 *
 * @param pvParameters Pointer to the global SharedRobotState.
 */
void adc_task(void *pvParameters) {
  SharedRobotState *state = static_cast<SharedRobotState *>(pvParameters);
  ESP_LOGI(TAG, "ADC Task Started");

  while (true) {
    adc_driver.process_dma_events(portMAX_DELAY);

    adc_sensor_data_t adc_data;
    adc_driver.read_sensor_data(&adc_data);

    portENTER_CRITICAL(&state->spinlock);
    for (int i = 0; i < ROBOT_NUM_SENSORS; i++) {
      state->adc_raw[i] = adc_data.raw[i];
    }
    portEXIT_CRITICAL(&state->spinlock);
  }
}

/**
 * @brief HX711 Loadcell Task.
 *
 * Polls the HX711 sensor for weight data at a defined interval and updates the
 * global state. Runs on Core 1 with medium priority (3).
 *
 * @param pvParameters Pointer to the global SharedRobotState.
 */
void loadcell_task(void *pvParameters) {
  SharedRobotState *state = static_cast<SharedRobotState *>(pvParameters);
  ESP_LOGI(TAG, "Loadcell Task Started");

  while (true) {
    float weight = loadcell.get_units(1);

    portENTER_CRITICAL(&state->spinlock);
    state->loadcell_weight = weight;
    portEXIT_CRITICAL(&state->spinlock);

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/**
 * @brief Wi-Fi Toggle Task.
 *
 * Monitors a physical switch to toggle Wi-Fi connection on/off dynamically.
 * Runs on Core 1 with low priority (2).
 *
 * @param pvParameters Not used.
 */
void wifi_toggle_task(void *pvParameters) {
  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = (1ULL << PIN_WIFI_SWITCH);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&io_conf);

  bool wifi_is_running =
      true; // Wi-Fi is already started in app_main by wifi.init()

  while (true) {
    int switch_state = gpio_get_level(PIN_WIFI_SWITCH);

    // Switch connected to GND -> state 0 -> turn ON Wi-Fi
    if (switch_state == 0 && !wifi_is_running) {
      ESP_LOGI(TAG, "Switch ON: Starting Wi-Fi...");
      wifi.start();
      wifi_is_running = true;
    }
    // Switch disconnected -> state 1 (pull-up) -> turn OFF Wi-Fi
    else if (switch_state == 1 && wifi_is_running) {
      ESP_LOGI(TAG, "Switch OFF: Stopping Wi-Fi...");
      wifi.stop();
      wifi_is_running = false;
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

struct NvsPidData {
  float kp, kd, pid_tau;
  float kp_load1, kd_load1, pid_tau_load1;
  float kp_load2, kd_load2, pid_tau_load2;
  float sensor_weight_04, sensor_weight_13;
  float kp_l, ki_l, kd_l;
  float kp_r, ki_r, kd_r;
  float v_ref;
  float v_ref_turn;
};

struct NvsBlindData {
  bool fuzzy_mode;
  float blind_seg1_rpm_l;
  float blind_seg1_rpm_r;
  int64_t blind_seg1_pulses_l;
  int64_t blind_seg1_pulses_r;
  float blind_seg2_rpm_l;
  float blind_seg2_rpm_r;
  int64_t blind_seg2_pulses_l;
  int64_t blind_seg2_pulses_r;
  float blind_seg3_rpm_l;
  float blind_seg3_rpm_r;
};

/**
 * @brief Loads PID and Blind run parameters from NVS flash memory.
 *
 * @param state Reference to the global SharedRobotState where config will be
 * stored.
 */
void load_nvs_params(SharedRobotState &state) {
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Error (%s) opening NVS handle! Using defaults.",
             esp_err_to_name(err));
    return;
  }

  NvsPidData pid_data;
  size_t required_size = sizeof(NvsPidData);
  err = nvs_get_blob(my_handle, "pid_cfg", &pid_data, &required_size);
  if (err == ESP_OK) {
    state.physical_config.kp = pid_data.kp;
    state.physical_config.kd = pid_data.kd;
    state.physical_config.pid_tau = pid_data.pid_tau;
    state.physical_config.kp_load1 = pid_data.kp_load1;
    state.physical_config.kd_load1 = pid_data.kd_load1;
    state.physical_config.pid_tau_load1 = pid_data.pid_tau_load1;
    state.physical_config.kp_load2 = pid_data.kp_load2;
    state.physical_config.kd_load2 = pid_data.kd_load2;
    state.physical_config.pid_tau_load2 = pid_data.pid_tau_load2;
    state.physical_config.sensor_weight_04 = pid_data.sensor_weight_04;
    state.physical_config.sensor_weight_13 = pid_data.sensor_weight_13;
    state.physical_config.kp_l = pid_data.kp_l;
    state.physical_config.ki_l = pid_data.ki_l;
    state.physical_config.kd_l = pid_data.kd_l;
    state.physical_config.kp_r = pid_data.kp_r;
    state.physical_config.ki_r = pid_data.ki_r;
    state.physical_config.kd_r = pid_data.kd_r;
    state.physical_config.v_ref = pid_data.v_ref;
    state.physical_config.v_ref_turn = pid_data.v_ref_turn;
    ESP_LOGI(TAG, "Loaded PID params from NVS successfully.");
  } else {
    ESP_LOGW(TAG, "Failed to load PID params from NVS (using defaults).");
  }

  NvsBlindData blind_data;
  size_t blind_size = sizeof(NvsBlindData);
  err = nvs_get_blob(my_handle, "blind_cfg", &blind_data, &blind_size);
  if (err == ESP_OK) {
    state.track_config.fuzzy_mode = blind_data.fuzzy_mode;
    state.track_config.blind_seg1_rpm_l = blind_data.blind_seg1_rpm_l;
    state.track_config.blind_seg1_rpm_r = blind_data.blind_seg1_rpm_r;
    state.track_config.blind_seg1_pulses_l = blind_data.blind_seg1_pulses_l;
    state.track_config.blind_seg1_pulses_r = blind_data.blind_seg1_pulses_r;
    state.track_config.blind_seg2_rpm_l = blind_data.blind_seg2_rpm_l;
    state.track_config.blind_seg2_rpm_r = blind_data.blind_seg2_rpm_r;
    state.track_config.blind_seg2_pulses_l = blind_data.blind_seg2_pulses_l;
    state.track_config.blind_seg2_pulses_r = blind_data.blind_seg2_pulses_r;
    state.track_config.blind_seg3_rpm_l = blind_data.blind_seg3_rpm_l;
    state.track_config.blind_seg3_rpm_r = blind_data.blind_seg3_rpm_r;
    ESP_LOGI(TAG, "Loaded Blind Run / Fuzzy params from NVS successfully.");
  } else {
    ESP_LOGW(TAG, "Failed to load Blind Run params from NVS (using defaults).");
  }

  nvs_close(my_handle);
}

/**
 * @brief Saves the current PID and physical parameters to NVS flash memory.
 *
 * @param state Reference to the global SharedRobotState containing the config.
 */
void save_nvs_params(const SharedRobotState &state) {
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) opening NVS handle to save!",
             esp_err_to_name(err));
    return;
  }

  NvsPidData pid_data = {
      .kp = state.physical_config.kp,
      .kd = state.physical_config.kd,
      .pid_tau = state.physical_config.pid_tau,
      .kp_load1 = state.physical_config.kp_load1,
      .kd_load1 = state.physical_config.kd_load1,
      .pid_tau_load1 = state.physical_config.pid_tau_load1,
      .kp_load2 = state.physical_config.kp_load2,
      .kd_load2 = state.physical_config.kd_load2,
      .pid_tau_load2 = state.physical_config.pid_tau_load2,
      .sensor_weight_04 = state.physical_config.sensor_weight_04,
      .sensor_weight_13 = state.physical_config.sensor_weight_13,
      .kp_l = state.physical_config.kp_l,
      .ki_l = state.physical_config.ki_l,
      .kd_l = state.physical_config.kd_l,
      .kp_r = state.physical_config.kp_r,
      .ki_r = state.physical_config.ki_r,
      .kd_r = state.physical_config.kd_r,
      .v_ref = state.physical_config.v_ref,
      .v_ref_turn = state.physical_config.v_ref_turn};

  err = nvs_set_blob(my_handle, "pid_cfg", &pid_data, sizeof(NvsPidData));
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Saved PID params to NVS blob.");
  }

  NvsBlindData blind_data = {
      .fuzzy_mode = state.track_config.fuzzy_mode,
      .blind_seg1_rpm_l = state.track_config.blind_seg1_rpm_l,
      .blind_seg1_rpm_r = state.track_config.blind_seg1_rpm_r,
      .blind_seg1_pulses_l = state.track_config.blind_seg1_pulses_l,
      .blind_seg1_pulses_r = state.track_config.blind_seg1_pulses_r,
      .blind_seg2_rpm_l = state.track_config.blind_seg2_rpm_l,
      .blind_seg2_rpm_r = state.track_config.blind_seg2_rpm_r,
      .blind_seg2_pulses_l = state.track_config.blind_seg2_pulses_l,
      .blind_seg2_pulses_r = state.track_config.blind_seg2_pulses_r,
      .blind_seg3_rpm_l = state.track_config.blind_seg3_rpm_l,
      .blind_seg3_rpm_r = state.track_config.blind_seg3_rpm_r};

  err = nvs_set_blob(my_handle, "blind_cfg", &blind_data, sizeof(NvsBlindData));
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Saved Blind Run params to NVS blob.");
  }

  err = nvs_commit(my_handle);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "NVS commit successful.");
  }
  nvs_close(my_handle);
}

/**
 * @brief UDP Receiver Task for remote PID tuning.
 *
 * Listens for JSON-formatted UDP packets to update PID tuning dynamically in
 * RAM, or save the current configuration to NVS. Runs on Core 0.
 *
 * @note Dynamic Allocation Exception: This task uses `cJSON_Parse` which
 * performs `malloc()` internally. Although dynamic allocation is generally
 * forbidden in continuous loops by project rules, it is permitted here because
 * this task only executes its allocation path upon receiving specific manual
 * tuning packets, which occurs very rarely.
 *
 * @param pvParameters Pointer to the global SharedRobotState.
 */
void udp_receiver_task(void *pvParameters) {
  SharedRobotState *state = static_cast<SharedRobotState *>(pvParameters);
  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (sock < 0) {
    ESP_LOGE(TAG, "Unable to create UDP receiver socket");
    vTaskDelete(NULL);
    return;
  }

  struct sockaddr_in dest_addr = {};
  dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(UDP_LISTEN_PORT);

  int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  if (err < 0) {
    ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    close(sock);
    vTaskDelete(NULL);
    return;
  }

  struct timeval tv = {.tv_sec = 1, .tv_usec = 0};
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  ESP_LOGI(TAG, "UDP receiver listening on port %d", UDP_LISTEN_PORT);
  char rx_buffer[512];

  while (true) {
    if (!wifi.is_connected()) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    struct sockaddr_storage source_addr;
    socklen_t socklen = sizeof(source_addr);
    int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                       (struct sockaddr *)&source_addr, &socklen);

    if (len < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK && wifi.is_connected()) {
        ESP_LOGW(TAG, "recvfrom failed: errno %d", errno);
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    rx_buffer[len] = '\0';
    cJSON *json = cJSON_Parse(rx_buffer);
    if (json == NULL)
      continue;

    cJSON *cmd = cJSON_GetObjectItemCaseSensitive(json, "cmd");
    if (cJSON_IsString(cmd) && (cmd->valuestring != NULL)) {
      bool is_running = false;
      portENTER_CRITICAL(&state->spinlock);
      is_running = state->system_running;
      portEXIT_CRITICAL(&state->spinlock);

      if (strcmp(cmd->valuestring, "start") == 0) {
        portENTER_CRITICAL(&state->spinlock);
        state->system_running = true;
        state->soft_stop_request = false;
        portEXIT_CRITICAL(&state->spinlock);
        ESP_LOGI(TAG, "UDP Command: START");
      } else if (strcmp(cmd->valuestring, "stop") == 0) {
        portENTER_CRITICAL(&state->spinlock);
        state->soft_stop_request = true;
        state->test_mode_active = false;
        portEXIT_CRITICAL(&state->spinlock);
        ESP_LOGI(TAG, "UDP Command: STOP (Soft brake requested)");
      } else if (strcmp(cmd->valuestring, "tune") == 0) {
        if (is_running) {
          ESP_LOGW(TAG, "Cannot tune PID while system is running!");
        } else {
          cJSON *pid_l = cJSON_GetObjectItemCaseSensitive(json, "pid_L");
          cJSON *pid_r = cJSON_GetObjectItemCaseSensitive(json, "pid_R");
          cJSON *pid_t = cJSON_GetObjectItemCaseSensitive(json, "pid_T");
          cJSON *pid_t_1 = cJSON_GetObjectItemCaseSensitive(json, "pid_T_1");
          cJSON *pid_t_2 = cJSON_GetObjectItemCaseSensitive(json, "pid_T_2");
          cJSON *sensor_weights =
              cJSON_GetObjectItemCaseSensitive(json, "sensor_weights");
          cJSON *v_ref_json = cJSON_GetObjectItemCaseSensitive(json, "v_ref");
          cJSON *v_ref_turn_json =
              cJSON_GetObjectItemCaseSensitive(json, "v_ref_turn");

          bool update_l = false, update_r = false, update_t = false,
               update_t_1 = false, update_t_2 = false, update_w = false,
               update_v = false, update_v_turn = false;
          float l_p, l_i, l_d;
          float r_p, r_i, r_d;
          float t_p, t_d, t_tau = -1.0f;
          float t1_p, t1_d, t1_tau = -1.0f;
          float t2_p, t2_d, t2_tau = -1.0f;
          float w_04, w_13;
          float v_ref, v_ref_turn;

          if (cJSON_IsArray(pid_l) && cJSON_GetArraySize(pid_l) == 3) {
            l_p = cJSON_GetArrayItem(pid_l, 0)->valuedouble;
            l_i = cJSON_GetArrayItem(pid_l, 1)->valuedouble;
            l_d = cJSON_GetArrayItem(pid_l, 2)->valuedouble;
            update_l = true;
          }
          if (cJSON_IsArray(pid_r) && cJSON_GetArraySize(pid_r) == 3) {
            r_p = cJSON_GetArrayItem(pid_r, 0)->valuedouble;
            r_i = cJSON_GetArrayItem(pid_r, 1)->valuedouble;
            r_d = cJSON_GetArrayItem(pid_r, 2)->valuedouble;
            update_r = true;
          }
          if (cJSON_IsArray(pid_t) && cJSON_GetArraySize(pid_t) >= 2) {
            t_p = cJSON_GetArrayItem(pid_t, 0)->valuedouble;
            t_d = cJSON_GetArrayItem(pid_t, 1)->valuedouble;
            if (cJSON_GetArraySize(pid_t) >= 3) {
              t_tau = cJSON_GetArrayItem(pid_t, 2)->valuedouble;
            }
            update_t = true;
          }
          if (cJSON_IsArray(pid_t_1) && cJSON_GetArraySize(pid_t_1) >= 2) {
            t1_p = cJSON_GetArrayItem(pid_t_1, 0)->valuedouble;
            t1_d = cJSON_GetArrayItem(pid_t_1, 1)->valuedouble;
            if (cJSON_GetArraySize(pid_t_1) >= 3) {
              t1_tau = cJSON_GetArrayItem(pid_t_1, 2)->valuedouble;
            }
            update_t_1 = true;
          }
          if (cJSON_IsArray(pid_t_2) && cJSON_GetArraySize(pid_t_2) >= 2) {
            t2_p = cJSON_GetArrayItem(pid_t_2, 0)->valuedouble;
            t2_d = cJSON_GetArrayItem(pid_t_2, 1)->valuedouble;
            if (cJSON_GetArraySize(pid_t_2) >= 3) {
              t2_tau = cJSON_GetArrayItem(pid_t_2, 2)->valuedouble;
            }
            update_t_2 = true;
          }
          if (cJSON_IsArray(sensor_weights) &&
              cJSON_GetArraySize(sensor_weights) == 2) {
            w_04 = cJSON_GetArrayItem(sensor_weights, 0)->valuedouble;
            w_13 = cJSON_GetArrayItem(sensor_weights, 1)->valuedouble;
            update_w = true;
          }
          if (cJSON_IsNumber(v_ref_json)) {
            v_ref = v_ref_json->valuedouble;
            update_v = true;
          }
          if (cJSON_IsNumber(v_ref_turn_json)) {
            v_ref_turn = v_ref_turn_json->valuedouble;
            update_v_turn = true;
          }

          cJSON *fuzzy_json = cJSON_GetObjectItemCaseSensitive(json, "fuzzy");
          cJSON *blind_s1 = cJSON_GetObjectItemCaseSensitive(json, "blind_s1");
          cJSON *blind_s2 = cJSON_GetObjectItemCaseSensitive(json, "blind_s2");
          cJSON *blind_s3 = cJSON_GetObjectItemCaseSensitive(json, "blind_s3");

          bool update_fuzzy = false, update_bs1 = false, update_bs2 = false,
               update_bs3 = false;
          bool fuzzy_val = false;
          float bs1_rl = 0.0f, bs1_rr = 0.0f;
          int64_t bs1_pl = 0, bs1_pr = 0;
          float bs2_rl = 0.0f, bs2_rr = 0.0f;
          int64_t bs2_pl = 0, bs2_pr = 0;
          float bs3_rl = 0.0f, bs3_rr = 0.0f;

          if (cJSON_IsBool(fuzzy_json)) {
            fuzzy_val = cJSON_IsTrue(fuzzy_json);
            update_fuzzy = true;
          }
          if (cJSON_IsArray(blind_s1) && cJSON_GetArraySize(blind_s1) == 4) {
            bs1_rl = cJSON_GetArrayItem(blind_s1, 0)->valuedouble;
            bs1_rr = cJSON_GetArrayItem(blind_s1, 1)->valuedouble;
            bs1_pl = (int64_t)cJSON_GetArrayItem(blind_s1, 2)->valuedouble;
            bs1_pr = (int64_t)cJSON_GetArrayItem(blind_s1, 3)->valuedouble;
            update_bs1 = true;
          }
          if (cJSON_IsArray(blind_s2) && cJSON_GetArraySize(blind_s2) == 4) {
            bs2_rl = cJSON_GetArrayItem(blind_s2, 0)->valuedouble;
            bs2_rr = cJSON_GetArrayItem(blind_s2, 1)->valuedouble;
            bs2_pl = (int64_t)cJSON_GetArrayItem(blind_s2, 2)->valuedouble;
            bs2_pr = (int64_t)cJSON_GetArrayItem(blind_s2, 3)->valuedouble;
            update_bs2 = true;
          }
          if (cJSON_IsArray(blind_s3) && cJSON_GetArraySize(blind_s3) == 2) {
            bs3_rl = cJSON_GetArrayItem(blind_s3, 0)->valuedouble;
            bs3_rr = cJSON_GetArrayItem(blind_s3, 1)->valuedouble;
            update_bs3 = true;
          }

          portENTER_CRITICAL(&state->spinlock);
          if (update_fuzzy) {
            state->track_config.fuzzy_mode = fuzzy_val;
          }
          if (update_bs1) {
            state->track_config.blind_seg1_rpm_l = bs1_rl;
            state->track_config.blind_seg1_rpm_r = bs1_rr;
            state->track_config.blind_seg1_pulses_l = bs1_pl;
            state->track_config.blind_seg1_pulses_r = bs1_pr;
          }
          if (update_bs2) {
            state->track_config.blind_seg2_rpm_l = bs2_rl;
            state->track_config.blind_seg2_rpm_r = bs2_rr;
            state->track_config.blind_seg2_pulses_l = bs2_pl;
            state->track_config.blind_seg2_pulses_r = bs2_pr;
          }
          if (update_bs3) {
            state->track_config.blind_seg3_rpm_l = bs3_rl;
            state->track_config.blind_seg3_rpm_r = bs3_rr;
          }
          if (update_l) {
            state->physical_config.kp_l = l_p;
            state->physical_config.ki_l = l_i;
            state->physical_config.kd_l = l_d;
          }
          if (update_r) {
            state->physical_config.kp_r = r_p;
            state->physical_config.ki_r = r_i;
            state->physical_config.kd_r = r_d;
          }
          if (update_t) {
            state->physical_config.kp = t_p;
            state->physical_config.kd = t_d;
            if (t_tau >= 0.0f) {
              state->physical_config.pid_tau = t_tau;
            }
          }
          if (update_t_1) {
            state->physical_config.kp_load1 = t1_p;
            state->physical_config.kd_load1 = t1_d;
            if (t1_tau >= 0.0f) {
              state->physical_config.pid_tau_load1 = t1_tau;
            }
          }
          if (update_t_2) {
            state->physical_config.kp_load2 = t2_p;
            state->physical_config.kd_load2 = t2_d;
            if (t2_tau >= 0.0f) {
              state->physical_config.pid_tau_load2 = t2_tau;
            }
          }
          if (update_w) {
            state->physical_config.sensor_weight_04 = w_04;
            state->physical_config.sensor_weight_13 = w_13;
          }
          if (update_v) {
            state->physical_config.v_ref = v_ref;
          }
          if (update_v_turn) {
            state->physical_config.v_ref_turn = v_ref_turn;
          }
          portEXIT_CRITICAL(&state->spinlock);

          if (update_l) {
            pid_left.set_tunings(l_p, l_i, l_d);
          }
          if (update_r) {
            pid_right.set_tunings(r_p, r_i, r_d);
          }

          ESP_LOGI(TAG, "Applied new PID tunings to RAM.");
        }
      } else if (strcmp(cmd->valuestring, "save") == 0) {
        SharedRobotState local_state;
        portENTER_CRITICAL(&state->spinlock);
        local_state = *state;
        portEXIT_CRITICAL(&state->spinlock);
        save_nvs_params(local_state);
      } else if (strcmp(cmd->valuestring, "manual_drive") == 0) {
        if (is_running) {
          ESP_LOGW(TAG,
                   "Cannot manual drive while autonomous system is running!");
        } else {
          cJSON *rpm_l = cJSON_GetObjectItemCaseSensitive(json, "rpm_l");
          cJSON *rpm_r = cJSON_GetObjectItemCaseSensitive(json, "rpm_r");

          portENTER_CRITICAL(&state->spinlock);
          if (cJSON_IsNumber(rpm_l)) {
            state->manual_cmd_l = rpm_l->valuedouble;
          }
          if (cJSON_IsNumber(rpm_r)) {
            state->manual_cmd_r = rpm_r->valuedouble;
          }
          portEXIT_CRITICAL(&state->spinlock);
        }
      } else if (strcmp(cmd->valuestring, "test_pid") == 0) {
        cJSON *rpm_l = cJSON_GetObjectItemCaseSensitive(json, "rpm_l");
        cJSON *rpm_r = cJSON_GetObjectItemCaseSensitive(json, "rpm_r");

        portENTER_CRITICAL(&state->spinlock);
        if (cJSON_IsNumber(rpm_l)) {
          state->test_target_rpm_l = rpm_l->valuedouble;
        }
        if (cJSON_IsNumber(rpm_r)) {
          state->test_target_rpm_r = rpm_r->valuedouble;
        }
        state->test_mode_active = true;
        state->system_running = true;
        state->soft_stop_request = false;
        portEXIT_CRITICAL(&state->spinlock);

        ESP_LOGI(TAG, "UDP Command: TEST_PID");
      }
    }
    cJSON_Delete(json);
  }
}

/**
 * @brief Application Main Entry Point.
 *
 * Initializes NVS, Wi-Fi, Drivers, and orchestrates the creation of all
 * FreeRTOS tasks.
 */
extern "C" void app_main() {
  ESP_LOGI(TAG, "Initializing Orchestrator...");

#if USE_LAPTOP_HOTSPOT
  wifi_manager::WifiConfig wifi_cfg(wifi_manager::WifiMode::MODE_STA,
                                    WIFI_HOTSPOT_SSID, WIFI_HOTSPOT_PASS);
#else
  wifi_manager::WifiConfig wifi_cfg(wifi_manager::WifiMode::MODE_STA,
                                    WIFI_ROUTER_SSID, WIFI_ROUTER_PASS);
#endif
  ESP_ERROR_CHECK(wifi.init(wifi_cfg));
  // wifi.start(); // Handled dynamically by wifi_toggle_task

  // Load PID parameters from NVS
  load_nvs_params(robot_state);

  adc_dma_config_t adc_cfg = {};
  adc_cfg.sample_freq_hz = 20000;
  adc_cfg.dma_frame_size = 256;
  adc_cfg.channel_map[SENSOR_01] = PIN_ADC_SENSOR_02; // Swapped 1 and 2
  adc_cfg.channel_map[SENSOR_02] = PIN_ADC_SENSOR_01; // Swapped 1 and 2
  adc_cfg.channel_map[SENSOR_03] = PIN_ADC_SENSOR_04; // Swapped 3 and 4
  adc_cfg.channel_map[SENSOR_04] = PIN_ADC_SENSOR_03; // Swapped 3 and 4
  adc_cfg.channel_map[SENSOR_05] = PIN_ADC_SENSOR_05;
  ESP_ERROR_CHECK(adc_driver.init(adc_cfg));
  ESP_ERROR_CHECK(adc_driver.start());

  tb6612_config_t mleft_cfg = {.pwm_gpio = PIN_MOTOR_L_PWM,
                               .in1_gpio = PIN_MOTOR_L_IN1,
                               .in2_gpio = PIN_MOTOR_L_IN2,
                               .enc_a_gpio = PIN_ENC_L_A,
                               .enc_b_gpio = PIN_ENC_L_B,
                               .pwm_freq_hz = 1000,
                               .encoder_ppr = PHYS_ENCODER_PPR,
                               .pcnt_high_limit = 30000,
                               .pcnt_low_limit = -30000};
  ESP_ERROR_CHECK(motor_left.init(mleft_cfg));

  tb6612_config_t mright_cfg = {.pwm_gpio = PIN_MOTOR_R_PWM,
                                .in1_gpio = PIN_MOTOR_R_IN1,
                                .in2_gpio = PIN_MOTOR_R_IN2,
                                .enc_a_gpio = PIN_ENC_R_A,
                                .enc_b_gpio = PIN_ENC_R_B,
                                .pwm_freq_hz = 1000,
                                .encoder_ppr = PHYS_ENCODER_PPR,
                                .pcnt_high_limit = 30000,
                                .pcnt_low_limit = -30000};
  ESP_ERROR_CHECK(motor_right.init(mright_cfg));

  velocity_pid_config_t pid_cfg_l = {
      .kp = robot_state.physical_config.kp_l,
      .ki = robot_state.physical_config.ki_l,
      .kd = robot_state.physical_config.kd_l,
      .out_max = 85.0f,      // Maximum PWM duty cycle limit is 85%
      .out_min = 0.0f,       // Enforce >= 0 for safety against reverse pulses
      .integral_max = 85.0f, // Corresponding Integral Anti-Windup limit
      .max_accel_units_s2 = 5000.0f};
  velocity_pid_config_t pid_cfg_r = {
      .kp = robot_state.physical_config.kp_r,
      .ki = robot_state.physical_config.ki_r,
      .kd = robot_state.physical_config.kd_r,
      .out_max = 85.0f,
      .out_min = 0.0f, // Enforce >= 0 for safety against reverse pulses
      .integral_max = 85.0f,
      .max_accel_units_s2 = 5000.0f};
  pid_left.init(pid_cfg_l);
  pid_right.init(pid_cfg_r);

  if (loadcell.begin(128)) {
    ESP_LOGI(TAG, "Waiting for HX711 to stabilize...");
    // 1. Wait for physical strain gauge to settle after power up
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 2. Discard first few unstable readings
    loadcell.read_average(3);

    // 3. Tare with higher sample count for better zeroing accuracy (20 samples
    // = ~2 seconds)
    ESP_LOGI(TAG, "Taring loadcell...");
    loadcell.tare(20);

    // New scale factor based on the average:
    // (184.5 * 2280) / 1 = 420,660 raw/kg
    // (371.5 * 2280) / 2 = 423,510 raw/kg
    // (556.5 * 2280) / 3 = 422,940 raw/kg
    // Average = 422,370 raw / 1 kg.
    // To output in grams (1kg = 1000g), Scale = 422,370 / 1000 = 422.37.
    // Minus sign (-) to invert negative values to positive.
    loadcell.set_scale(-422.37f);
    ESP_LOGI(TAG, "Loadcell ready.");
  }

  // Orchestrate tasks with explicit Core Pinning and Priorities
  xTaskCreatePinnedToCore(adc_task, "ADC_Task", 4096, &robot_state, 6, nullptr,
                          1);
  xTaskCreatePinnedToCore(loadcell_task, "Load_Task", 4096, &robot_state, 3,
                          nullptr, 1);
  xTaskCreatePinnedToCore(motion_task_routine, "Motion_Task", 4096,
                          &robot_state, 5, nullptr, 1);
  xTaskCreatePinnedToCore(telemetry_task_routine, "Tele_Task", 4096,
                          &robot_state, 2, nullptr, 0);
  xTaskCreatePinnedToCore(wifi_toggle_task, "Wifi_Toggle", 2048, nullptr, 2,
                          nullptr, 1);
  xTaskCreatePinnedToCore(udp_receiver_task, "UDP_Recv", 4096, &robot_state, 1,
                          nullptr, 0);

  ESP_LOGI(TAG, "Tasks deployed. Yielding app_main.");
}
