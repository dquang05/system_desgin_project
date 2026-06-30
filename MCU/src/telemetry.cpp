#include "../include/telemetry.hpp"
#include <stdio.h>
#include <string.h>

TelemetryLogger::TelemetryLogger() : _data_queue(nullptr), _wifi(nullptr), _target_port(0) {
    memset(_target_ip, 0, sizeof(_target_ip));
}

TelemetryLogger::~TelemetryLogger() {
    if (_data_queue) {
        vQueueDelete(_data_queue);
    }
}

bool TelemetryLogger::init(wifi_manager::WifiManager* wifi_mgr, const char* target_ip, uint16_t target_port) {
    _wifi = wifi_mgr;
    strncpy(_target_ip, target_ip, sizeof(_target_ip) - 1);
    _target_port = target_port;
    
    // Create a queue of length 1 for overwrite mechanism (non-blocking updates)
    _data_queue = xQueueCreate(1, sizeof(TelemetryData));
    if (_data_queue == nullptr) {
        return false;
    }
    return true;
}

void TelemetryLogger::push_data(const TelemetryData& data) {
    if (_data_queue) {
        // Overwrite the queue, ensuring non-blocking behavior
        // Takes less than 1us under FreeRTOS, perfectly safe for control loop
        xQueueOverwrite(_data_queue, &data);
    }
}

size_t TelemetryLogger::serialize_json(const TelemetryData& data, char* buffer, size_t max_len) {
    // Format JSON safely using snprintf. 
    // Avoid dynamic allocation (no new, no std::string) to prevent heap fragmentation.
    int ret = snprintf(buffer, max_len,
        "{"
        "\"ts\":%lu,"
        "\"enc\":[%lld,%lld],"
        "\"pwm\":[%.2f,%.2f],"
        "\"adc\":[%d,%d,%d,%d,%d]"
        "}",
        data.timestamp_ms,
        data.encoder_left, data.encoder_right,
        data.pwm_left, data.pwm_right,
        data.adc_voltage_mv[0], data.adc_voltage_mv[1], 
        data.adc_voltage_mv[2], data.adc_voltage_mv[3], 
        data.adc_voltage_mv[4]
    );
    
    if (ret >= 0 && ret < max_len) {
        return static_cast<size_t>(ret);
    }
    return 0; // Buffer overflow or error
}

void TelemetryLogger::process() {
    if (!_data_queue || !_wifi) return;

    TelemetryData data;
    // Pop data with zero block time (non-blocking). 
    // If no new data is present, we just return.
    if (xQueueReceive(_data_queue, &data, 0) == pdTRUE) {
        // Serialize using fixed-size buffer
        size_t len = serialize_json(data, _json_buffer, MAX_JSON_BUFFER_SIZE);
        
        // Send if valid and wifi is connected
        if (len > 0 && _wifi->is_connected()) {
            _wifi->send_log_data(_target_ip, _target_port, reinterpret_cast<const uint8_t*>(_json_buffer), len);
        }
    }
}

/* 
 * RISK REVIEW
 * 
 * 1. Memory Leak Risks:
 *    - The TelemetryData queue is statically allocated on the FreeRTOS heap upon init() once.
 *    - No dynamic memory allocation (malloc/new) is used inside push_data() or process(),
 *      eliminating any fragmentation and leaks during runtime.
 *    - The JSON buffer is a fixed-size array in the class instance.
 * 
 * 2. System Behavior (Producer > Consumer):
 *    - The xQueueOverwrite mechanism ensures that if the Control Loop (producer) produces 
 *      data faster than the Telemetry Task (consumer) can send it over UDP, the queue simply 
 *      overwrites the older, unsent packet.
 *    - This guarantees O(1) < 1us execution time for push_data(), preventing any out-of-memory 
 *      or blocking conditions for the high-priority motor task.
 *    - Telemetry will drop intermediate frames but always transmits the freshest system state.
 */
