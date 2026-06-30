#include "../lib/wifi_manager/wifi_manager.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <stdio.h>

static const char *TAG = "WIFI_TEST";

// Chân GPIO an toàn (không dùng touch/ADC quan trọng)
// GPIO 25 có thể kích hoạt điện trở kéo xuống (pull-down) an toàn
#define WIFI_CONTROL_GPIO GPIO_NUM_25

extern "C" void app_main() {
  ESP_LOGI(TAG, "Starting Wi-Fi Manager Test...");

  // 1. Khởi tạo Wi-Fi
  static wifi_manager::WifiManager wifi;
  wifi_manager::WifiConfig config(wifi_manager::WifiMode::MODE_STA,
                                  "VIETTEL",
                                  "0906608600");
  wifi.init(config);

  // Dừng Wi-Fi ngay từ đầu (mặc định), đợi tín hiệu từ GPIO
  wifi.stop();

  // 2. Cấu hình GPIO input với điện trở kéo xuống (mặc định là LOW)
  gpio_config_t io_conf = {};
  io_conf.intr_type =
      GPIO_INTR_DISABLE; // Dùng kiểu polling cho đơn giản & tránh nhiễu nút bấm
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << WIFI_CONTROL_GPIO);
  io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE; // Kéo xuống LOW
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

  ESP_LOGI(TAG,
           "Wi-Fi is currently STOPPED. Set GPIO %d to HIGH to start Wi-Fi.",
           WIFI_CONTROL_GPIO);

  // 3. Vòng lặp chính kiểm tra tín hiệu
  bool last_state = false;
  uint32_t last_log_time = 0;

  while (true) {
    int current_level = gpio_get_level(WIFI_CONTROL_GPIO);
    bool current_state = (current_level == 1);

    if (current_state != last_state) {
      last_state = current_state;

      if (current_state) {
        ESP_LOGI(TAG, "GPIO %d is HIGH -> BẬT Wi-Fi", WIFI_CONTROL_GPIO);
        wifi.start();
      } else {
        ESP_LOGI(TAG, "GPIO %d is LOW -> TẮT Wi-Fi", WIFI_CONTROL_GPIO);
        wifi.stop();
      }
    }

    // Nếu Wi-Fi đang bật, kiểm tra kết nối và gửi log mỗi 2s
    if (current_state) {
      // Kiểm tra trạng thái bằng hàm không log (tránh spam CPU/UART)
      if (wifi.is_connected()) {
        uint32_t current_time = pdTICKS_TO_MS(xTaskGetTickCount());
        // Kiểm tra xem đã qua 2000ms (2s) chưa
        if (current_time - last_log_time >= 2000) {
          last_log_time = current_time;

          // Demo gửi dữ liệu định kỳ
          const char *dummy_log = "{\"sensor\": \"temp\", \"value\": 25.4}";
          bool sent =
              wifi.send_log_data("192.168.1.14", 54321,
                                 reinterpret_cast<const uint8_t *>(dummy_log),
                                 std::strlen(dummy_log));
          if (sent) {
            ESP_LOGI(TAG, "Log data sent successfully.");
          } else {
            ESP_LOGE(TAG, "Failed to send log data.");
          }
        }
      }
    }

    // Trễ 100ms: Vừa tiết kiệm CPU, vừa dùng làm debouncer chống nhiễu phím
    // cứng
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
