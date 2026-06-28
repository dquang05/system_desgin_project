#include "app_services.hpp"
#include "../lib/wifi_manager/wifi_manager.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include <cstring>

namespace app_services {

static const char *TAG = "APP_SERVICES";
#define WIFI_CONTROL_GPIO GPIO_NUM_25

// Maintain the library objects here at the application layer
static wifi_manager::WifiManager wifi;

// ---------------------------------------------------------
// Wi-Fi Service Implementation
// ---------------------------------------------------------
static void wifi_task(void *pvParameters) {
  ESP_LOGI(TAG, "Wi-Fi Service Task Started.");

  wifi_manager::WifiConfig config(wifi_manager::WifiMode::MODE_STA,
                                  wifi_manager::DEFAULT_WIFI_SSID,
                                  wifi_manager::DEFAULT_WIFI_PASSWORD);
  wifi.init(config);
  wifi.stop();

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

    if (current_state && wifi.is_connected()) {
      uint32_t current_time = pdTICKS_TO_MS(xTaskGetTickCount());
      if (current_time - last_log_time >= 2000) {
        last_log_time = current_time;
        const char *dummy_log = "{\"sensor\": \"temp\", \"value\": 25.4}";
        bool sent = wifi.send_log_data(
            "192.168.1.15", 8080, reinterpret_cast<const uint8_t *>(dummy_log),
            std::strlen(dummy_log));
        if (sent) {
          ESP_LOGI(TAG, "Log data sent successfully.");
        } else {
          ESP_LOGE(TAG, "Failed to send log data.");
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void wifi_service_start(UBaseType_t priority) {
  xTaskCreate(wifi_task, "wifi_task", 4096, nullptr, priority, nullptr);
}

// ---------------------------------------------------------
// Sensor Service Implementation
// ---------------------------------------------------------
static void sensor_task(void *pvParameters) {
  ESP_LOGI(TAG, "Sensor Service Task Started.");
  while (true) {
    // Read sensor data and process it
    // Example: adc_driver.read_sensor_data(&data);

    // Wait for 1000ms
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void sensor_service_start(UBaseType_t priority) {
  xTaskCreate(sensor_task, "sensor_task", 4096, nullptr, priority, nullptr);
}

// ---------------------------------------------------------
// Hardware Initialization
// ---------------------------------------------------------
void hardware_init() {
  ESP_LOGI(TAG, "Initializing Hardware Subsystems...");

  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << WIFI_CONTROL_GPIO);
  io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);
}

} // namespace app_services
