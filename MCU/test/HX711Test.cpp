/**
 * @file HX711Test.cpp
 * @brief Test for HX711 Loadcell Amplifier
 *
 * =========================================================================
 * PHYSICAL WIRING GUIDE (ESP32 -> HX711 -> Loadcell):
 * =========================================================================
 * [ESP32 Pin]      [HX711 Pin]      [Loadcell Wire]
 * -------------------------------------------------------------------------
 * GPIO 35   <----- DT (Data)        -
 * GPIO 13   -----> SCK (Clock)      -
 *
 * 3.3V      -----> VCC / VDD        -
 * GND       -----> GND              -
 *
 * -         <----- E+ (Excitation+) RED
 * -         <----- E- (Excitation-) BLACK
 * -         <----- A- (Signal -)    WHITE
 * -         <----- A+ (Signal +)    GREEN
 */

#include "../lib/loadcell_hx711/loadcell_hx711.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <unity.h>

static const char *TAG = "HX711_TEST";

void test_hx711_read_weight() {
  ESP_LOGI(TAG, "Initializing HX711...");
  LoadcellHX711 scale(GPIO_NUM_35, GPIO_NUM_13);

  bool init_res = scale.begin(128);
  TEST_ASSERT_TRUE(init_res);
  ESP_LOGI(TAG, "HX711 init successful");

  ESP_LOGI(TAG, "Taring scale (10 samples)...");
  scale.tare(10);
  ESP_LOGI(TAG, "Tare complete");

  scale.set_scale(2280.f);
  ESP_LOGI(TAG, "Scale factor set to 2280.0");

  for (int i = 0; i < 5; i++) {
    float weight = scale.get_units(5);
    ESP_LOGI(TAG, "Weight: %.2f", weight);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Required entry point for PlatformIO testing
extern "C" void app_main() {
  // Short delay to allow Serial Monitor to connect and capture early logs
  vTaskDelay(pdMS_TO_TICKS(2000));

  UNITY_BEGIN();
  RUN_TEST(test_hx711_read_weight);
  UNITY_END();
}
