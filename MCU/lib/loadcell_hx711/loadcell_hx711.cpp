/**
 * @file loadcell_hx711.cpp
 * @brief Implementation of the HX711 Loadcell Amplifier logic.
 */
#include "loadcell_hx711.hpp"
#include "rom/ets_sys.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "HX711";

LoadcellHX711::LoadcellHX711(gpio_num_t dt_pin, gpio_num_t sck_pin) 
    : _dt_pin(dt_pin), _sck_pin(sck_pin), _gain(128), _offset(0), _scale(1.0f) {
}

bool LoadcellHX711::begin(uint8_t gain) {
    _hw_init_pins();
    
    // Set gain logic: 
    // Gain 128 = 25 pulses (1 extra)
    // Gain 32  = 26 pulses (2 extra)
    // Gain 64  = 27 pulses (3 extra)
    if (gain == 128) _gain = 1;
    else if (gain == 32) _gain = 2;
    else if (gain == 64) _gain = 3;
    else _gain = 1; // Default to 128 if invalid

    // Power up logic by pulling SCK low
    _hw_set_sck(false);

    // Initial read to set the gain properly for subsequent readings
    ESP_LOGI(TAG, "Initializing HX711 with Gain %d", (gain == 128 || gain == 64 || gain == 32) ? gain : 128);
    read_raw();
    return true;
}

void LoadcellHX711::_hw_init_pins() {
    gpio_reset_pin(_sck_pin);
    gpio_set_direction(_sck_pin, GPIO_MODE_OUTPUT);
    
    gpio_reset_pin(_dt_pin);
    gpio_set_direction(_dt_pin, GPIO_MODE_INPUT);
}

void LoadcellHX711::_hw_set_sck(bool level) {
    gpio_set_level(_sck_pin, level ? 1 : 0);
}

bool LoadcellHX711::_hw_read_dt() {
    return gpio_get_level(_dt_pin) == 1;
}

void LoadcellHX711::_hw_delay_us(uint32_t us) {
    ets_delay_us(us);
}

void LoadcellHX711::_hw_yield_cpu() {
    vTaskDelay(pdMS_TO_TICKS(1)); // Minimum delay to yield CPU and avoid TWDT
}

uint64_t LoadcellHX711::_hw_get_time_us() {
    return esp_timer_get_time();
}

long LoadcellHX711::read_raw() {
    // Wait for DT to go LOW (Data Ready)
    // With 10Hz setting, it can take up to 100ms. Timeout set to 150ms.
    uint64_t start_time = _hw_get_time_us();
    while (_hw_read_dt()) {
        if ((_hw_get_time_us() - start_time) > HX711_TIMEOUT_US) {
            // Timeout occurred
            ESP_LOGE(TAG, "Timeout waiting for DT to go LOW");
            return 0; // Return 0 to avoid hanging the system
        }
        _hw_yield_cpu(); // Yield to prevent TWDT crash (Critical for ESP32)
    }

    // Read 24 bits
    long count = 0;
    for (int i = 0; i < 24; ++i) {
        _hw_set_sck(true);
        // HX711 max SCK high time is 50us (otherwise power down). Minimum is 0.2us.
        _hw_delay_us(1); 
        count = count << 1;
        _hw_set_sck(false);
        _hw_delay_us(1);
        
        if (_hw_read_dt()) {
            count++;
        }
    }

    // Send extra SCK pulses to set gain for the next reading
    for (int i = 0; i < _gain; ++i) {
        _hw_set_sck(true);
        _hw_delay_us(1);
        _hw_set_sck(false);
        _hw_delay_us(1);
    }

    /**
     * @note Sign-extension logic:
     * The HX711 returns a 24-bit 2's complement number.
     * If the 24th bit (bit 23) is 1, the number is negative and we must 
     * pad the upper 8 bits (of our 32-bit container) with 1s to preserve the sign.
     */
    if (count & 0x800000) { // If bit 23 is 1 (negative)
        count |= 0xFF000000; // Pad the upper 8 bits with 1s
    } else {
        count &= 0x00FFFFFF; // Redundant but safe
    }

    return count;
}

long LoadcellHX711::read_average(uint8_t times) {
    if (times == 0) return 0;
    long sum = 0;
    for (uint8_t i = 0; i < times; ++i) {
        sum += read_raw();
    }
    return sum / times;
}

void LoadcellHX711::tare(uint8_t times) {
    long sum = read_average(times);
    _offset = sum;
}

float LoadcellHX711::get_units(uint8_t times) {
    long val = read_average(times);
    return static_cast<float>(val - _offset) / _scale;
}

void LoadcellHX711::set_scale(float scale) {
    if (scale == 0.0f) _scale = 1.0f; // Prevent division by zero
    else _scale = scale;
}

// RISK REVIEW:
// 1. Timing: `read_raw` uses `ets_delay_us(1)` which blocks CPU briefly. This is necessary for HX711 signaling, but if higher priority ISRs interrupt this heavily, it could violate the HX711 50us maximum SCK high time (causing a power-down reset).
// 2. Timeout: The timeout is set to 150ms. If the loadcell wire breaks, it safely returns 0, but will block the calling thread for 150ms each time it's called.
// 3. Thread Safety: The methods are not intrinsically thread-safe. If multiple RTOS tasks access the same `LoadcellHX711` instance, a mutex is required at the application level.
// 4. Data Validity: Returning `0` on timeout might be ambiguous if the actual weight reading is exactly `0`. A separate `is_ready()` or error struct could be more robust.
