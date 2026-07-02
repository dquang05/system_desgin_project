#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../include/motion_task.hpp"
#include "../include/shared_state.hpp"
#include "../include/telemetry_task.hpp"

#include "../lib/adc_dma/adc_dma.hpp"
#include "../lib/tb6612_encoder/tb6612_encoder.hpp"
#include "../lib/velocity_pid/velocity_pid.hpp"
#include "../lib/wifi_manager/wifi_manager.hpp"

static const char *TAG = "ORCHESTRATOR";

// ==================== HARDWARE PIN MAPPING ====================
// --- TB6612FNG Motor Driver Pins ---
constexpr int PIN_MOTOR_L_PWM = 17;
constexpr int PIN_MOTOR_L_IN1 = 16;
constexpr int PIN_MOTOR_L_IN2 = 4;

constexpr int PIN_MOTOR_R_PWM = 23;
constexpr int PIN_MOTOR_R_IN1 = 19;
constexpr int PIN_MOTOR_R_IN2 = 18;

// --- JGB37-520 Encoder Pins (Must support interrupts) ---
constexpr int PIN_ENC_L_A = 25;
constexpr int PIN_ENC_L_B = 26;

constexpr int PIN_ENC_R_A = 27;
constexpr int PIN_ENC_R_B = 14;

// --- 5-Channel Line Sensor Pins ---
// Mapped internally via ADC Channels in adc_dma.hpp:
// SENSOR 1: GPIO36 (ADC1_CH0) - VP
// SENSOR 2: GPIO39 (ADC1_CH3) - VN
// SENSOR 3: GPIO32 (ADC1_CH4)
// SENSOR 4: GPIO33 (ADC1_CH5)
// SENSOR 5: GPIO34 (ADC1_CH6)

// System-wide configurations
// const char* UDP_TARGET_IP = "192.168.1.14";
const char *UDP_TARGET_IP = "192.168.0.116";
extern const uint16_t UDP_TARGET_PORT;
const uint16_t UDP_TARGET_PORT = 54321;

// Global Shared State
SharedRobotState robot_state = {.spinlock = portMUX_INITIALIZER_UNLOCKED,
                                .adc_raw = {0},
                                .encoder_left = 0,
                                .encoder_right = 0,
                                .pwm_left = 0.0f,
                                .pwm_right = 0.0f,
                                .target_rpm_left = 0.0f,
                                .target_rpm_right = 0.0f,
                                .actual_rpm_left = 0.0f,
                                .actual_rpm_right = 0.0f};

// Global Drivers (Workers)
wifi_manager::WifiManager wifi;
EspAdcDmaDriver adc_driver;
Tb6612Encoder motor_left;
Tb6612Encoder motor_right;
VelocityPid pid_left;
VelocityPid pid_right;

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

extern "C" void app_main() {
  ESP_LOGI(TAG, "Initializing Orchestrator...");

  wifi_manager::WifiConfig wifi_cfg(wifi_manager::WifiMode::MODE_STA,
                                    "TP-Link_C718", "20017231");
  ESP_ERROR_CHECK(wifi.init(wifi_cfg));
  wifi.start();

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
                               .encoder_ppr = 341,
                               .pcnt_high_limit = 30000,
                               .pcnt_low_limit = -30000};
  ESP_ERROR_CHECK(motor_left.init(mleft_cfg));

  tb6612_config_t mright_cfg = {.pwm_gpio = PIN_MOTOR_R_PWM,
                                .in1_gpio = PIN_MOTOR_R_IN1,
                                .in2_gpio = PIN_MOTOR_R_IN2,
                                .enc_a_gpio = PIN_ENC_R_A,
                                .enc_b_gpio = PIN_ENC_R_B,
                                .pwm_freq_hz = 20000,
                                .encoder_ppr = 341,
                                .pcnt_high_limit = 30000,
                                .pcnt_low_limit = -30000};
  ESP_ERROR_CHECK(motor_right.init(mright_cfg));

  velocity_pid_config_t pid_cfg = {
      .kp = 1.0f,
      .ki = 0.0f,
      .kd = 0.0f,
      .out_max = 85.0f, // Giới hạn điện áp cấp (PWM) max là 85%
      .out_min = -85.0f,
      .integral_max = 85.0f, // Giới hạn Windup tích phân tương ứng
      .max_accel_units_s2 = 1000.0f};
  pid_left.init(pid_cfg);
  pid_right.init(pid_cfg);

  // Orchestrate tasks with explicit Core Pinning and Priorities
  xTaskCreatePinnedToCore(adc_task, "ADC_Task", 4096, &robot_state, 6, nullptr,
                          1);
  xTaskCreatePinnedToCore(motion_task_routine, "Motion_Task", 4096,
                          &robot_state, 5, nullptr, 1);
  xTaskCreatePinnedToCore(telemetry_task_routine, "Tele_Task", 4096,
                          &robot_state, 2, nullptr, 0);

  ESP_LOGI(TAG, "Tasks deployed. Yielding app_main.");
}
