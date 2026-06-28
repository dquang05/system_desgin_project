#include "motion_task.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>

static const char* TAG = "MOTION_TASK";

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Definition of global variables
wheel_vel_t global_target_vel = {0.0f, 0.0f};
SemaphoreHandle_t global_vel_mutex = nullptr;

static void motion_control_task(void *pvParameters) {
    if (global_vel_mutex == nullptr) {
        ESP_LOGE(TAG, "Mutex not initialized! Task terminating.");
        vTaskDelete(nullptr);
        return;
    }

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t freq_ticks = pdMS_TO_TICKS(10); // 10ms loop
    
    int64_t last_time_us = esp_timer_get_time();
    int64_t last_pulse_left = 0;
    int64_t last_pulse_right = 0;

    // Get initial pulse counts
    motor_left.get_pulse_count(last_pulse_left);
    motor_right.get_pulse_count(last_pulse_right);

    while (true) {
        // a) Lock Mutex, copy global_target_vel, unlock
        wheel_vel_t local_target_vel = {0.0f, 0.0f};
        if (xSemaphoreTake(global_vel_mutex, portMAX_DELAY) == pdTRUE) {
            local_target_vel = global_target_vel;
            xSemaphoreGive(global_vel_mutex);
        }

        // b) Update PID setpoints
        pid_left.set_target_velocity(local_target_vel.left);
        pid_right.set_target_velocity(local_target_vel.right);

        // c) Read current pulse counts
        int64_t current_pulse_left = 0;
        int64_t current_pulse_right = 0;
        motor_left.get_pulse_count(current_pulse_left);
        motor_right.get_pulse_count(current_pulse_right);

        // Calculate delta time
        int64_t current_time_us = esp_timer_get_time();
        int64_t delta_time_us = current_time_us - last_time_us;
        
        // Prevent division by zero if task runs too fast
        if (delta_time_us <= 0) {
            vTaskDelayUntil(&last_wake_time, freq_ticks);
            continue;
        }
        
        float dt_s = static_cast<float>(delta_time_us) / 1000000.0f;

        // d) Calculate current RPM
        float rpm_left = 0.0f;
        float rpm_right = 0.0f;
        motor_left.get_current_rpm(current_pulse_left, last_pulse_left, delta_time_us, rpm_left);
        motor_right.get_current_rpm(current_pulse_right, last_pulse_right, delta_time_us, rpm_right);

        // e) Convert RPM to linear velocity (mm/s)
        float current_vel_left = (rpm_left / 60.0f) * 2.0f * M_PI * WHEEL_RADIUS_MM;
        float current_vel_right = (rpm_right / 60.0f) * 2.0f * M_PI * WHEEL_RADIUS_MM;

        // f) Compute PID duty cycles
        float duty_left = pid_left.compute(current_vel_left, dt_s);
        float duty_right = pid_right.compute(current_vel_right, dt_s);

        // g) Update motor duty cycles
        motor_left.set_duty_cycle(duty_left);
        motor_right.set_duty_cycle(duty_right);

        // h) Update state for next cycle
        last_pulse_left = current_pulse_left;
        last_pulse_right = current_pulse_right;
        last_time_us = current_time_us;

        // i) Delay until next cycle
        vTaskDelayUntil(&last_wake_time, freq_ticks);
    }
}

void motion_task_start() {
    global_vel_mutex = xSemaphoreCreateMutex();
    if (global_vel_mutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create global_vel_mutex");
        return;
    }

    BaseType_t res = xTaskCreatePinnedToCore(
        motion_control_task,
        "motion_ctrl_tsk",
        4096,           // Stack size
        nullptr,        // Parameters
        5,              // High priority
        nullptr,        // Task handle
        1               // Run on Core 1
    );

    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create motion_control_task");
    } else {
        ESP_LOGI(TAG, "Motion control task started successfully");
    }
}

// RISK REVIEW:
// - Deadlock Risk: There is a risk of deadlock or severe latency if other tasks hold `global_vel_mutex` for too long. Ensure that any task writing to `global_target_vel` acquires the mutex, copies the data quickly, and releases it immediately without blocking.
// - Tick Rate Recommendation: It is highly recommended to increase `CONFIG_FREERTOS_HZ` to 1000 in menuconfig. The default 100Hz tick rate means the minimum delay resolution is 10ms. For a precise 10ms control loop using `vTaskDelayUntil`, a 1000Hz tick rate (1ms resolution) ensures much tighter scheduling and reduces jitter in the PID control loop.
