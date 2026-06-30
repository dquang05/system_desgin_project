#pragma once

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "../lib/wifi_manager/wifi_manager.hpp"

#define MAX_JSON_BUFFER_SIZE 256

struct TelemetryData {
    int64_t encoder_left;
    int64_t encoder_right;
    float pwm_left;
    float pwm_right;
    uint32_t adc_raw[5];
    int adc_voltage_mv[5];
    uint32_t timestamp_ms;
};

class TelemetryLogger {
public:
    TelemetryLogger();
    ~TelemetryLogger();

    // Initialize the telemetry module with queue and wifi reference
    bool init(wifi_manager::WifiManager* wifi_mgr, const char* target_ip, uint16_t target_port);

    // Called by Control Loop: push data non-blocking
    void push_data(const TelemetryData& data);

    // Called by Telemetry Task: pop data, format to JSON, and send
    void process();

private:
    QueueHandle_t _data_queue;
    wifi_manager::WifiManager* _wifi;
    char _json_buffer[MAX_JSON_BUFFER_SIZE];
    char _target_ip[16];
    uint16_t _target_port;
    
    // Internal method for serialization
    size_t serialize_json(const TelemetryData& data, char* buffer, size_t max_len);
};
