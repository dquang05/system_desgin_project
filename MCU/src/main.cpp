#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_services.hpp"
#include "motion_task.hpp"

static const char *TAG = "MAIN";

// Instantiate hardware and control objects required by motion_task
Tb6612Encoder motor_left;
Tb6612Encoder motor_right;
VelocityPid pid_left;
VelocityPid pid_right;

extern "C" void app_main() {
    ESP_LOGI(TAG, "System Booting Up...");

    // 1. Khởi tạo toàn bộ phần cứng (GPIO, ADC, Motor...)
    app_services::hardware_init();

    // Khởi tạo cấu hình mặc định (Dummy config để pass build)
    tb6612_config_t motor_cfg = {};
    motor_left.init(motor_cfg);
    motor_right.init(motor_cfg);

    velocity_pid_config_t pid_cfg = {1.0f, 0.1f, 0.01f, 100.0f, -100.0f, 100.0f, 10.0f};
    pid_left.init(pid_cfg);
    pid_right.init(pid_cfg);

    // 2. Chạy các Service chạy ngầm
    app_services::wifi_service_start(5);   // Priority 5
    app_services::sensor_service_start(4); // Priority 4

    // Khởi động motion task
    motion_task_start();

    ESP_LOGI(TAG, "All services started successfully. Main task is now going to idle.");

    // 3. Main task có thể ngủ hoặc làm việc khác
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
