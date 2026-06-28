/**
 * @file MotorTest.cpp
 * @brief Test for TB6612 Motor & Encoder Driver
 *
 * =========================================================================
 * PHYSICAL WIRING GUIDE (ESP32 -> TB6612 -> Motor with Encoder):
 * =========================================================================
 * [ESP32 Pin]      [TB6612 Pin]     [Motor / Encoder]
 * -------------------------------------------------------------------------
 * GPIO 14   -----> PWMA             -
 * GPIO 26   -----> AIN1             -
 * GPIO 27   -----> AIN2             -
 *
 * GPIO 34   <----- -                Encoder Phase A (e.g. Yellow Wire)
 * GPIO 35   <----- -                Encoder Phase B (e.g. Green Wire)
 *
 * 3.3V      -----> VCC (Logic)      Encoder VCC (e.g. Blue or Red Wire)
 * VMOTOR    -----> VM (Motor Pwr)   -
 * GND       -----> GND              Encoder GND (e.g. Black Wire)
 *
 * -         -----> AO1              Motor Terminal + (Red)
 * -         -----> AO2              Motor Terminal - (Black)
 * =========================================================================
 * Note: GPIO 34, 35 are input-only pins, perfect for encoder reading.
 */

#include "../lib/tb6612_encoder/tb6612_encoder.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <unity.h>

static const char *TAG = "MOTOR_TEST";

void test_motor_forward_backward_and_encoder_read() {
  Tb6612Encoder motor;

  tb6612_config_t config = {
      .pwm_gpio = 14,
      .in1_gpio = 26,
      .in2_gpio = 27,
      .enc_a_gpio = 34,
      .enc_b_gpio = 35,
      .pwm_freq_hz = 20000,
      .encoder_ppr =
          341, // Example: 341 PPR. Vui lòng sửa lại đúng loại motor của bạn!
      .pcnt_high_limit = 30000,
      .pcnt_low_limit = -30000};

  ESP_LOGI(TAG, "Initializing TB6612 motor driver...");
  esp_err_t err = motor.init(config);
  TEST_ASSERT_EQUAL(ESP_OK, err);

  ESP_LOGI(TAG, "Motor initialized. Running FORWARD at 50%% duty cycle...");
  motor.set_duty_cycle(50.0f);

  // Read values for 3 seconds
  for (int i = 0; i < 30; i++) {
    int64_t pulse_count = 0;
    float current_rpm = 0.0f;

    motor.get_pulse_count(pulse_count);
    motor.get_current_rpm(current_rpm);

    ESP_LOGI(TAG, "[FWD | Duty: 50%%] Pulses: %lld | RPM: %.2f", pulse_count,
             current_rpm);
    vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz polling
  }

  ESP_LOGI(TAG, "Stopping motor...");
  motor.set_duty_cycle(0.0f);
  vTaskDelay(pdMS_TO_TICKS(1000));

  ESP_LOGI(TAG, "Running BACKWARD at -30%% duty cycle...");
  motor.set_duty_cycle(-30.0f);

  // Read values for 3 seconds
  for (int i = 0; i < 30; i++) {
    int64_t pulse_count = 0;
    float current_rpm = 0.0f;

    motor.get_pulse_count(pulse_count);
    motor.get_current_rpm(current_rpm);

    ESP_LOGI(TAG, "[BWD | Duty: -30%%] Pulses: %lld | RPM: %.2f", pulse_count,
             current_rpm);
    vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz polling
  }

  ESP_LOGI(TAG, "Stopping and de-initializing motor...");
  motor.set_duty_cycle(0.0f);
}

// Bắt buộc trong PlatformIO khi chạy test
extern "C" void app_main_motor() {
  // Delay xíu để mở Serial Monitor k kịp vẫn đọc được log
  vTaskDelay(pdMS_TO_TICKS(2000));

  UNITY_BEGIN();
  RUN_TEST(test_motor_forward_backward_and_encoder_read);
  UNITY_END();
}
