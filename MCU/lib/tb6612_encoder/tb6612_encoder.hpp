#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/pulse_cnt.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

constexpr float TB6612_MAX_RPM = 333.0f;

struct tb6612_config_t {
    int pwm_gpio;
    int in1_gpio;
    int in2_gpio;
    int enc_a_gpio;
    int enc_b_gpio;
    uint32_t pwm_freq_hz;
    uint32_t encoder_ppr; // Pulses Per Revolution (e.g. 341, 1000)
    int pcnt_high_limit;  // Recommended: 30000
    int pcnt_low_limit;   // Recommended: -30000
};

class Tb6612Encoder {
public:
    Tb6612Encoder();
    ~Tb6612Encoder();

    esp_err_t init(const tb6612_config_t& config);
    esp_err_t set_duty_cycle(float duty_cycle_percent);
    esp_err_t set_speed_rpm(float rpm);
    esp_err_t get_pulse_count(int64_t& out_pulse_count);
    esp_err_t get_current_rpm(float& out_rpm);

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
    
    int64_t _last_rpm_calc_pulses;
    int64_t _last_rpm_calc_time_us;
    
    bool _is_initialized;

    static bool _pcnt_on_reach_cb(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);
};
