/**
 * @file tb6612_encoder.hpp
 * @brief TB6612FNG Motor Driver and Quadrature Encoder control library.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/pulse_cnt.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

/**
 * @brief Maximum theoretical RPM for the attached motor.
 */
constexpr float TB6612_MAX_RPM = 333.0f;

/**
 * @brief Configuration structure for the TB6612 motor and encoder.
 */
struct tb6612_config_t {
    int pwm_gpio;         /**< GPIO pin for PWM output */
    int in1_gpio;         /**< GPIO pin for IN1 direction control */
    int in2_gpio;         /**< GPIO pin for IN2 direction control */
    int enc_a_gpio;       /**< GPIO pin for Encoder Phase A */
    int enc_b_gpio;       /**< GPIO pin for Encoder Phase B */
    uint32_t pwm_freq_hz; /**< PWM frequency in Hz */
    uint32_t encoder_ppr; /**< Pulses Per Revolution (e.g., 341, 1000) */
    int pcnt_high_limit;  /**< PCNT high counter limit (e.g., 30000) */
    int pcnt_low_limit;   /**< PCNT low counter limit (e.g., -30000) */
};

/**
 * @brief TB6612 Motor Driver and Quadrature Encoder class.
 */
class Tb6612Encoder {
public:
    Tb6612Encoder();
    ~Tb6612Encoder();

    /**
     * @brief Initializes the motor driver and pulse counter (PCNT) peripherals.
     * @param config The configuration object.
     * @return ESP_OK on success.
     */
    esp_err_t init(const tb6612_config_t& config);

    /**
     * @brief Sets the motor duty cycle percentage.
     * @param duty_cycle_percent Value from -100.0 (full reverse) to 100.0 (full forward).
     * @return ESP_OK on success.
     */
    esp_err_t set_duty_cycle(float duty_cycle_percent);

    /**
     * @brief Sets the motor speed using open-loop RPM mapping.
     * @param rpm Target RPM.
     * @return ESP_OK on success.
     */
    esp_err_t set_speed_rpm_openloop(float rpm);

    /**
     * @brief Gets the total accumulated pulse count from the encoder.
     * @param out_pulse_count Reference to store the pulse count.
     * @return ESP_OK on success.
     */
    esp_err_t get_pulse_count(int64_t& out_pulse_count);

    /**
     * @brief Calculate current RPM based on pulse and time deltas.
     * 
     * @note Quantization Noise Warning: At high sampling frequencies (very small delta_time_us), 
     * the number of delta_pulses captured will be very small and quantized. This causes a 
     * "staircase" effect (quantization noise) on the calculated RPM. 
     * It is the responsibility of the caller (e.g., PID module at a higher layer) to apply 
     * a Low-Pass Filter to smooth out this noise.
     * 
     * @param current_pulses Current pulse count.
     * @param last_pulses Pulse count from the previous sampling period.
     * @param delta_time_us Time elapsed between samples in microseconds.
     * @param out_rpm Reference to store the calculated RPM.
     * @return ESP_OK on success.
     */
    esp_err_t get_current_rpm(int64_t current_pulses, int64_t last_pulses, int64_t delta_time_us, float& out_rpm);

private:
    mcpwm_timer_handle_t _timer;
    mcpwm_oper_handle_t _oper;
    mcpwm_cmpr_handle_t _cmpr;
    mcpwm_gen_handle_t _gen;
    
    pcnt_unit_handle_t _pcnt_unit;
    pcnt_channel_handle_t _pcnt_chan_a;
    pcnt_channel_handle_t _pcnt_chan_b;
    
    int _in1_gpio;
    int _in2_gpio;
    uint32_t _pwm_period_ticks;
    uint32_t _encoder_ppr;
    
    portMUX_TYPE _spinlock;
    int64_t _accumulated_pulses;
    
    bool _is_initialized;

    static bool _pcnt_on_reach_cb(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);
};
