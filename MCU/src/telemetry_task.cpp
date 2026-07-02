#include "../include/telemetry_task.hpp"
#include "../include/shared_state.hpp"
#include "../lib/wifi_manager/wifi_manager.hpp"
#include <esp_timer.h>
#include <cstdio>

extern wifi_manager::WifiManager wifi;
extern const char* UDP_TARGET_IP;
extern const uint16_t UDP_TARGET_PORT;

void telemetry_task_routine(void *pvParameters) {
    SharedRobotState* state = static_cast<SharedRobotState*>(pvParameters);
    const TickType_t freq_ticks = pdMS_TO_TICKS(50); // 20Hz Logging Rate
    TickType_t last_wake_time = xTaskGetTickCount();
    char json_buf[256];

    while (true) {
        // Read isolated snapshot
        SharedRobotState local_state;
        portENTER_CRITICAL(&state->spinlock);
        local_state = *state;
        portEXIT_CRITICAL(&state->spinlock);

        // Serialize data
        int len = snprintf(json_buf, sizeof(json_buf),
            "{\"ts\":%lu,\"enc\":[%lld,%lld],\"pwm\":[%.2f,%.2f],\"adc\":[%lu,%lu,%lu,%lu,%lu],\"rpm_tgt\":[%.2f,%.2f],\"rpm_act\":[%.2f,%.2f]}",
            (uint32_t)(esp_timer_get_time() / 1000ULL),
            local_state.encoder_left, local_state.encoder_right,
            local_state.pwm_left, local_state.pwm_right,
            local_state.adc_raw[0], local_state.adc_raw[1],
            local_state.adc_raw[2], local_state.adc_raw[3],
            local_state.adc_raw[4],
            local_state.target_rpm_left, local_state.target_rpm_right,
            local_state.actual_rpm_left, local_state.actual_rpm_right);

        // Decoupled hardware transmission
        if (len > 0 && wifi.is_connected()) {
            wifi.send_log_data(UDP_TARGET_IP, UDP_TARGET_PORT, reinterpret_cast<const uint8_t*>(json_buf), len);
        }

        vTaskDelayUntil(&last_wake_time, freq_ticks);
    }
}
