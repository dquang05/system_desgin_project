/**
 * @file main.cpp
 * @brief Main entry point and orchestrator for the ESP32 AMR firmware.
 * 
 * This file contains the FreeRTOS task definitions, global state management, 
 * and hardware initialization. It acts as the central coordinator for all 
 * decoupled library modules.
 */
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <lwip/sockets.h>
#include <cJSON.h>

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
    .loadcell_weight = 0.0f,
    .line_calib = {.x_max = {4095, 4095, 4095, 4095, 4095},
                   .x_min = {0, 0, 0, 0, 0},
                   .y_max = 1000,
                   .y_min = 0,
                   .line_coe_1 = 1.0f,
                   .line_coe_2 = 0.0f},
    .physical_config = {.wheel_base_mm = DEFAULT_PHYS_WHEEL_BASE_MM,
                        .wheel_radius_mm = DEFAULT_PHYS_WHEEL_RADIUS_MM,
                        .v_ref = 0.0f,
                        .kp = DEFAULT_KP,
                        .kd = DEFAULT_KD,
                        .pid_tau = DEFAULT_PID_TAU,
                        .kp_l = DEFAULT_KP_L,
                        .ki_l = DEFAULT_KI_L,
                        .kd_l = DEFAULT_KD_L,
                        .kp_r = DEFAULT_KP_R,
                        .ki_r = DEFAULT_KI_R,
                        .kd_r = DEFAULT_KD_R}};

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
 * Polls the HX711 sensor for weight data at a defined interval and updates the global state.
 * Runs on Core 1 with medium priority (3).
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

    vTaskDelay(pdMS_TO_TICKS(10));
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

  bool wifi_is_running = false; // Initially not started

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

/**
 * @brief Loads PID and physical parameters from NVS flash memory.
 * 
 * @param state Reference to the global SharedRobotState where config will be stored.
 */
void load_nvs_params(SharedRobotState &state) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error (%s) opening NVS handle! Using defaults.", esp_err_to_name(err));
        return;
    }

    size_t required_size = sizeof(RobotPhysicalConfig);
    err = nvs_get_blob(my_handle, "phys_cfg", &state.physical_config, &required_size);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Loaded PID params from NVS successfully.");
    } else {
        ESP_LOGW(TAG, "Failed to load PID params from NVS (using defaults).");
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
        ESP_LOGE(TAG, "Error (%s) opening NVS handle to save!", esp_err_to_name(err));
        return;
    }

    err = nvs_set_blob(my_handle, "phys_cfg", &state.physical_config, sizeof(RobotPhysicalConfig));
    if (err == ESP_OK) {
        err = nvs_commit(my_handle);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Saved PID params to NVS successfully.");
        }
    }
    nvs_close(my_handle);
}

/**
 * @brief UDP Receiver Task for remote PID tuning.
 * 
 * Listens for JSON-formatted UDP packets to update PID tuning dynamically in RAM,
 * or save the current configuration to NVS. Runs on Core 0.
 * 
 * @note Dynamic Allocation Exception: This task uses `cJSON_Parse` which performs
 *       `malloc()` internally. Although dynamic allocation is generally forbidden
 *       in continuous loops by project rules, it is permitted here because this task 
 *       only executes its allocation path upon receiving specific manual tuning packets,
 *       which occurs very rarely. 
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

    ESP_LOGI(TAG, "UDP receiver listening on port %d", UDP_LISTEN_PORT);
    char rx_buffer[512];

    while (true) {
        struct sockaddr_storage source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        rx_buffer[len] = '\0';
        cJSON *json = cJSON_Parse(rx_buffer);
        if (json == NULL) continue;

        cJSON *cmd = cJSON_GetObjectItemCaseSensitive(json, "cmd");
        if (cJSON_IsString(cmd) && (cmd->valuestring != NULL)) {
            if (strcmp(cmd->valuestring, "tune") == 0) {
                cJSON *pid_l = cJSON_GetObjectItemCaseSensitive(json, "pid_L");
                cJSON *pid_r = cJSON_GetObjectItemCaseSensitive(json, "pid_R");
                cJSON *pid_t = cJSON_GetObjectItemCaseSensitive(json, "pid_T");

                portENTER_CRITICAL(&state->spinlock);
                if (cJSON_IsArray(pid_l) && cJSON_GetArraySize(pid_l) == 3) {
                    state->physical_config.kp_l = cJSON_GetArrayItem(pid_l, 0)->valuedouble;
                    state->physical_config.ki_l = cJSON_GetArrayItem(pid_l, 1)->valuedouble;
                    state->physical_config.kd_l = cJSON_GetArrayItem(pid_l, 2)->valuedouble;
                    pid_left.set_tunings(state->physical_config.kp_l, state->physical_config.ki_l, state->physical_config.kd_l);
                }
                if (cJSON_IsArray(pid_r) && cJSON_GetArraySize(pid_r) == 3) {
                    state->physical_config.kp_r = cJSON_GetArrayItem(pid_r, 0)->valuedouble;
                    state->physical_config.ki_r = cJSON_GetArrayItem(pid_r, 1)->valuedouble;
                    state->physical_config.kd_r = cJSON_GetArrayItem(pid_r, 2)->valuedouble;
                    pid_right.set_tunings(state->physical_config.kp_r, state->physical_config.ki_r, state->physical_config.kd_r);
                }
                if (cJSON_IsArray(pid_t) && cJSON_GetArraySize(pid_t) == 3) {
                    state->physical_config.kp = cJSON_GetArrayItem(pid_t, 0)->valuedouble;
                    state->physical_config.kd = cJSON_GetArrayItem(pid_t, 1)->valuedouble;
                    state->physical_config.pid_tau = cJSON_GetArrayItem(pid_t, 2)->valuedouble;
                }
                portEXIT_CRITICAL(&state->spinlock);
                ESP_LOGI(TAG, "Applied new PID tunings to RAM.");
            } else if (strcmp(cmd->valuestring, "save") == 0) {
                SharedRobotState local_state;
                portENTER_CRITICAL(&state->spinlock);
                local_state = *state;
                portEXIT_CRITICAL(&state->spinlock);
                save_nvs_params(local_state);
            }
        }
        cJSON_Delete(json);
    }
}

/**
 * @brief Application Main Entry Point.
 * 
 * Initializes NVS, Wi-Fi, Drivers, and orchestrates the creation of all FreeRTOS tasks.
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
  adc_cfg.channel_map[SENSOR_01] = PIN_ADC_SENSOR_01;
  adc_cfg.channel_map[SENSOR_02] = PIN_ADC_SENSOR_02;
  adc_cfg.channel_map[SENSOR_03] = PIN_ADC_SENSOR_03;
  adc_cfg.channel_map[SENSOR_04] = PIN_ADC_SENSOR_04;
  adc_cfg.channel_map[SENSOR_05] = PIN_ADC_SENSOR_05;
  ESP_ERROR_CHECK(adc_driver.init(adc_cfg));
  ESP_ERROR_CHECK(adc_driver.start());

  tb6612_config_t mleft_cfg = {.pwm_gpio = PIN_MOTOR_L_PWM,
                               .in1_gpio = PIN_MOTOR_L_IN1,
                               .in2_gpio = PIN_MOTOR_L_IN2,
                               .enc_a_gpio = PIN_ENC_L_A,
                               .enc_b_gpio = PIN_ENC_L_B,
                               .pwm_freq_hz = 20000,
                               .encoder_ppr = 330,
                               .pcnt_high_limit = 30000,
                               .pcnt_low_limit = -30000};
  ESP_ERROR_CHECK(motor_left.init(mleft_cfg));

  tb6612_config_t mright_cfg = {.pwm_gpio = PIN_MOTOR_R_PWM,
                                .in1_gpio = PIN_MOTOR_R_IN1,
                                .in2_gpio = PIN_MOTOR_R_IN2,
                                .enc_a_gpio = PIN_ENC_R_A,
                                .enc_b_gpio = PIN_ENC_R_B,
                                .pwm_freq_hz = 20000,
                                .encoder_ppr = 330,
                                .pcnt_high_limit = 30000,
                                .pcnt_low_limit = -30000};
  ESP_ERROR_CHECK(motor_right.init(mright_cfg));

  velocity_pid_config_t pid_cfg_l = {
      .kp = robot_state.physical_config.kp_l,
      .ki = robot_state.physical_config.ki_l,
      .kd = robot_state.physical_config.kd_l,
      .out_max = 85.0f, // Maximum PWM duty cycle limit is 85%
      .out_min = -85.0f,
      .integral_max = 85.0f, // Corresponding Integral Anti-Windup limit
      .max_accel_units_s2 = 1000.0f};
  velocity_pid_config_t pid_cfg_r = {
      .kp = robot_state.physical_config.kp_r,
      .ki = robot_state.physical_config.ki_r,
      .kd = robot_state.physical_config.kd_r,
      .out_max = 85.0f,
      .out_min = -85.0f,
      .integral_max = 85.0f,
      .max_accel_units_s2 = 1000.0f};
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
  xTaskCreatePinnedToCore(wifi_toggle_task, "Wifi_Toggle", 2048,
                          nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(udp_receiver_task, "UDP_Recv", 4096,
                          &robot_state, 1, nullptr, 0);

  ESP_LOGI(TAG, "Tasks deployed. Yielding app_main.");
}
