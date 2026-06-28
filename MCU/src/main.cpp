#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_services.hpp"

static const char *TAG = "MAIN";

extern "C" void app_main() {
    ESP_LOGI(TAG, "System Booting Up...");

    // 1. Khởi tạo toàn bộ phần cứng (GPIO, ADC, Motor...)
    app_services::hardware_init();

    // 2. Chạy các Service chạy ngầm
    app_services::wifi_service_start(5);   // Priority 5
    app_services::sensor_service_start(4); // Priority 4

    ESP_LOGI(TAG, "All services started successfully. Main task is now going to idle.");

    // 3. Main task có thể ngủ hoặc làm việc khác
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
