#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

// Hardware and modules
#include "../include/telemetry.hpp"
#include "../lib/adc_dma/adc_dma.hpp"
#include "../lib/tb6612_encoder/tb6612_encoder.hpp"
#include "../lib/wifi_manager/wifi_manager.hpp"

static const char *TAG = "MAIN";

// Global Instances
wifi_manager::WifiManager wifi;
EspAdcDmaDriver adc_driver;
Tb6612Encoder motor_left;
Tb6612Encoder motor_right;
TelemetryLogger telemetry;

// Task Handles
TaskHandle_t motor_task_handle = nullptr;
TaskHandle_t telemetry_task_handle = nullptr;
TaskHandle_t adc_task_handle = nullptr;

// UDP Target Configuration (Change to match your UI server's IP and port)
constexpr char TARGET_IP[] = "192.168.1.14";
constexpr uint16_t TARGET_PORT = 54321;

void adc_task(void *pvParameters) {
  ESP_LOGI(TAG, "ADC Task Started");
  while (true) {
    // Kéo dữ liệu từ DMA RingBuffer liên tục
    adc_driver.process_dma_events(portMAX_DELAY);
  }
}

void motor_control_task(void *pvParameters) {
  ESP_LOGI(TAG, "Motor Control Task Started");
  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t interval = pdMS_TO_TICKS(10); // 10ms loop (100Hz)

  while (true) {
    // 1. Read Encoders
    int64_t enc_l = 0, enc_r = 0;
    motor_left.get_pulse_count(enc_l);
    motor_right.get_pulse_count(enc_r);

    // 2. Read ADC
    adc_sensor_data_t adc_data;
    adc_driver.read_sensor_data(&adc_data);

    // 3. Process Control Logic (e.g. PID)
    // Dummy values for demonstration
    float pwm_l = 30.0f;
    float pwm_r = 30.0f;
    motor_left.set_duty_cycle(pwm_l);
    motor_right.set_duty_cycle(pwm_r);

    // 4. Push Telemetry Data
    TelemetryData tdata = {};
    tdata.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    tdata.encoder_left = enc_l;
    tdata.encoder_right = enc_r;
    tdata.pwm_left = pwm_l;
    tdata.pwm_right = pwm_r;
    for (int i = 0; i < NUM_SENSORS; ++i) {
      tdata.adc_raw[i] = adc_data.raw[i];
      tdata.adc_voltage_mv[i] = adc_data.voltage_mv[i];
    }

    // Non-blocking push, takes < 1us
    telemetry.push_data(tdata);

    // 5. Block until next cycle
    vTaskDelayUntil(&last_wake_time, interval);
  }
}

void telemetry_task(void *pvParameters) {
  ESP_LOGI(TAG, "Telemetry Task Started");
  const TickType_t interval = pdMS_TO_TICKS(50); // 20Hz logging rate

  while (true) {
    // Process telemetry queue and send via WiFi if available
    telemetry.process();

    // Delay to yield CPU and maintain the logging frequency
    vTaskDelay(interval);
  }
}

extern "C" void app_main() {
  ESP_LOGI(TAG, "Initializing System...");

  // 1. Initialize Wi-Fi
  wifi_manager::WifiConfig wifi_cfg(wifi_manager::WifiMode::MODE_STA, "VIETTEL",
                                    "0906608600");
  ESP_ERROR_CHECK(wifi.init(wifi_cfg));
  wifi.start();

  // 2. Initialize Telemetry Logger
  if (!telemetry.init(&wifi, TARGET_IP, TARGET_PORT)) {
    ESP_LOGE(TAG, "Failed to initialize Telemetry Logger");
  }

  // 3. Initialize ADC DMA
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

  // 4. Initialize Motors/Encoders
  tb6612_config_t mleft_cfg = {.pwm_gpio = 17,
                               .in1_gpio = 16,
                               .in2_gpio = 4, // Adjust to your actual layout
                               .enc_a_gpio = 35,
                               .enc_b_gpio = 34,
                               .pwm_freq_hz = 20000,
                               .encoder_ppr = 341,
                               .pcnt_high_limit = 30000,
                               .pcnt_low_limit = -30000};
  ESP_ERROR_CHECK(motor_left.init(mleft_cfg));

  tb6612_config_t mright_cfg = {.pwm_gpio = 23,
                                .in1_gpio = 19,
                                .in2_gpio = 18,
                                .enc_a_gpio = 39,
                                .enc_b_gpio = 36,
                                .pwm_freq_hz = 20000,
                                .encoder_ppr = 341,
                                .pcnt_high_limit = 30000,
                                .pcnt_low_limit = -30000};
  ESP_ERROR_CHECK(motor_right.init(mright_cfg));

  // 5. Create Tasks
  // ADC task gets highest priority to process DMA fast enough
  xTaskCreate(adc_task, "ADC_Task", 4096, nullptr, 6, &adc_task_handle);

  // Motor task gets high priority (5) to ensure precise, uninterrupted control loop
  xTaskCreate(motor_control_task, "Motor_Task", 4096, nullptr, 5,
              &motor_task_handle);

  // Telemetry task gets low priority (2) to avoid interfering with motor control
  xTaskCreate(telemetry_task, "Tele_Task", 4096, nullptr, 2,
              &telemetry_task_handle);

  ESP_LOGI(TAG, "Initialization Complete. Main task entering Idle.");
}
