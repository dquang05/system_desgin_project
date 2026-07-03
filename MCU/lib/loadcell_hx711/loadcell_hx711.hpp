#pragma once

#include <cstdint>
#include "driver/gpio.h"

class LoadcellHX711 {
public:
    /**
     * @brief Construct a new Loadcell HX711 object
     * 
     * @param dt_pin GPIO pin for Data (DT)
     * @param sck_pin GPIO pin for Clock (SCK)
     */
    LoadcellHX711(gpio_num_t dt_pin, gpio_num_t sck_pin);
    ~LoadcellHX711() = default;

    /**
     * @brief Initialize hardware pins and set initial gain
     * 
     * @param gain Valid values are 128, 64, or 32
     * @return true if initialization successful
     * @return false if initialization failed
     */
    bool begin(uint8_t gain = 128);

    /**
     * @brief Read 24-bit raw value from HX711
     * 
     * @return long 32-bit signed raw value, or 0 if timeout
     */
    long read_raw();

    /**
     * @brief Read average of multiple raw values
     * 
     * @param times Number of times to read and average
     * @return long Averaged raw value
     */
    long read_average(uint8_t times);

    /**
     * @brief Set the offset (tare weight)
     * 
     * @param times Number of readings to average for the offset
     */
    void tare(uint8_t times = 10);

    /**
     * @brief Get the actual physical units
     * 
     * @param times Number of readings to average
     * @return float Calculated weight based on offset and scale
     */
    float get_units(uint8_t times = 1);

    /**
     * @brief Set the scale factor
     * 
     * @param scale Scale factor (raw units per physical unit)
     */
    void set_scale(float scale);

protected:
    void _hw_init_pins();
    void _hw_set_sck(bool level);
    bool _hw_read_dt();
    void _hw_delay_us(uint32_t us);
    void _hw_yield_cpu();
    uint64_t _hw_get_time_us();

private:
    gpio_num_t _dt_pin;
    gpio_num_t _sck_pin;
    uint8_t _gain;
    long _offset;
    float _scale;
};
