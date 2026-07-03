#include "../lib/wifi_manager/wifi_manager.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <stdio.h>

static const char *TAG = "WIFI_TEST";

// Safe GPIO pin (avoids critical touch/ADC channels)
// GPIO 25 safely supports internal pull-down resistors
#define WIFI_CONTROL_GPIO GPIO_NUM_25

extern "C" void app_main() {
  ESP_LOGI(TAG, "Starting Wi-Fi Manager Test...");

  // 1. Initialize Wi-Fi
  static wifi_manager::WifiManager wifi;
  wifi_manager::WifiConfig config(wifi_manager::WifiMode::MODE_STA, "VIETTEL",
                                  "0906608600");
  // wifi.init(config);

  // Stop Wi-Fi initially, wait for GPIO signal
  wifi.stop();

  // 2. Configure GPIO input with pull-down resistor (default LOW)
  gpio_config_t io_conf = {};
  io_conf.intr_type =
      GPIO_INTR_DISABLE; // Polling mode for simplicity & debounce
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << WIFI_CONTROL_GPIO);
  io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE; // Pull down to LOW
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

  ESP_LOGI(TAG,
           "Wi-Fi is currently STOPPED. Set GPIO %d to HIGH to start Wi-Fi.",
           WIFI_CONTROL_GPIO);

  // 3. Main loop to check GPIO signal
  bool last_state = false;
  uint32_t last_log_time = 0;
  float x = 1.0f;
  while (true) {
    // int current_level = gpio_get_level(WIFI_CONTROL_GPIO);
    // bool current_state = (current_level == 1);

    // if (current_state != last_state) {
    //   last_state = current_state;

    //   if (current_state) {
    //     ESP_LOGI(TAG, "GPIO %d is HIGH -> Start Wi-Fi", WIFI_CONTROL_GPIO);
    //     // wifi.start();
    //   } else {
    //     ESP_LOGI(TAG, "GPIO %d is LOW -> Stop Wi-Fi", WIFI_CONTROL_GPIO);
    //     wifi.stop();
    //   }
    // }

    // // If Wi-Fi is enabled, check connection and send logs every 2s
    // if (current_state) {
    //   // Check status silently (avoids CPU/UART spam)
    //   if (wifi.is_connected()) {
    //     uint32_t current_time = pdTICKS_TO_MS(xTaskGetTickCount());
    //     // Check if 2000ms (2s) have passed
    //     if (current_time - last_log_time >= 2000) {
    //       last_log_time = current_time;

    //       // Demo periodic data transmission
    //       const char *dummy_log = "{\"sensor\": \"temp\", \"value\": 25.4}";
    //       bool sent =
    //           wifi.send_log_data("192.168.1.14", 54321,
    //                              reinterpret_cast<const uint8_t
    //                              *>(dummy_log), std::strlen(dummy_log));
    //       if (sent) {
    //         ESP_LOGI(TAG, "Log data sent successfully.");
    //       } else {
    //         ESP_LOGE(TAG, "Failed to send log data.");
    //       }
    //     }
    //   }
    // }

    // // 100ms delay: Saves CPU and acts as a hardware debouncer

    x = x * 1.0001f;
    ESP_LOGE(TAG, "Value of x: %f", x);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
