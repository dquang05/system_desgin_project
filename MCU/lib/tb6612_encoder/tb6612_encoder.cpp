#include "tb6612_encoder.hpp"
#include "esp_timer.h"
#include "esp_log.h"

static const char __attribute__((unused)) *TAG = "TB6612_ENC";

bool Tb6612Encoder::_pcnt_on_reach_cb(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
    Tb6612Encoder* motor = static_cast<Tb6612Encoder*>(user_ctx);
    portENTER_CRITICAL_ISR(&motor->_spinlock);
    motor->_accumulated_pulses += edata->watch_point_value;
    portEXIT_CRITICAL_ISR(&motor->_spinlock);
    return false; // No need to yield context
}

Tb6612Encoder::Tb6612Encoder() : 
    _timer(nullptr), _oper(nullptr), _cmpr(nullptr), _gen(nullptr),
    _pcnt_unit(nullptr), _pcnt_chan_a(nullptr), _pcnt_chan_b(nullptr),
    _in1_gpio(-1), _in2_gpio(-1), _pwm_period_ticks(0), _encoder_ppr(0),
    _spinlock(portMUX_INITIALIZER_UNLOCKED), _accumulated_pulses(0),
    _last_rpm_calc_pulses(0), _last_rpm_calc_time_us(0), _is_initialized(false) {
}

Tb6612Encoder::~Tb6612Encoder() {
    if (_is_initialized) {
        pcnt_unit_stop(_pcnt_unit);
        pcnt_unit_disable(_pcnt_unit);
        pcnt_del_channel(_pcnt_chan_a);
        pcnt_del_channel(_pcnt_chan_b);
        pcnt_del_unit(_pcnt_unit);

        mcpwm_timer_start_stop(_timer, MCPWM_TIMER_STOP_EMPTY);
        mcpwm_timer_disable(_timer);
        mcpwm_del_generator(_gen);
        mcpwm_del_comparator(_cmpr);
        mcpwm_del_operator(_oper);
        mcpwm_del_timer(_timer);
        
        gpio_reset_pin(static_cast<gpio_num_t>(_in1_gpio));
        gpio_reset_pin(static_cast<gpio_num_t>(_in2_gpio));
    }
}

esp_err_t Tb6612Encoder::init(const tb6612_config_t& config) {
    if (_is_initialized) return ESP_ERR_INVALID_STATE;

    _in1_gpio = config.in1_gpio;
    _in2_gpio = config.in2_gpio;
    _encoder_ppr = config.encoder_ppr;
    _accumulated_pulses = 0;
    _last_rpm_calc_pulses = 0;
    _last_rpm_calc_time_us = esp_timer_get_time();

    // 1. Initialize GPIOs for direction control (IN1, IN2)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config.in1_gpio) | (1ULL << config.in2_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(static_cast<gpio_num_t>(config.in1_gpio), 0));
    ESP_ERROR_CHECK(gpio_set_level(static_cast<gpio_num_t>(config.in2_gpio), 0));

    // 2. Initialize MCPWM v5 for PWM generation
    _pwm_period_ticks = 1000000 / config.pwm_freq_hz; // 1MHz resolution baseline
    mcpwm_timer_config_t timer_config = {};
    timer_config.group_id = 0;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = 1000000;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config.period_ticks = _pwm_period_ticks;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &_timer));

    mcpwm_operator_config_t oper_config = {};
    oper_config.group_id = 0;
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &_oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(_oper, _timer));

    mcpwm_comparator_config_t cmpr_config = {};
    cmpr_config.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(_oper, &cmpr_config, &_cmpr));

    mcpwm_generator_config_t gen_config = {};
    gen_config.gen_gpio_num = config.pwm_gpio;
    ESP_ERROR_CHECK(mcpwm_new_generator(_oper, &gen_config, &_gen));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(_gen,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(_gen,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, _cmpr, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP));

    // 3. Initialize PCNT v5 for Quadrature Encoder
    pcnt_unit_config_t unit_config = {};
    unit_config.low_limit = config.pcnt_low_limit;
    unit_config.high_limit = config.pcnt_high_limit;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &_pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {};
    filter_config.max_glitch_ns = 1000;
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(_pcnt_unit, &filter_config));

    // Channel A configuration
    pcnt_chan_config_t chan_a_config = {};
    chan_a_config.edge_gpio_num = config.enc_a_gpio;
    chan_a_config.level_gpio_num = config.enc_b_gpio;
    ESP_ERROR_CHECK(pcnt_new_channel(_pcnt_unit, &chan_a_config, &_pcnt_chan_a));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(_pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(_pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Channel B configuration
    pcnt_chan_config_t chan_b_config = {};
    chan_b_config.edge_gpio_num = config.enc_b_gpio;
    chan_b_config.level_gpio_num = config.enc_a_gpio;
    ESP_ERROR_CHECK(pcnt_new_channel(_pcnt_unit, &chan_b_config, &_pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(_pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(_pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Register watchpoints and callbacks
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(_pcnt_unit, config.pcnt_high_limit));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(_pcnt_unit, config.pcnt_low_limit));

    pcnt_event_callbacks_t cbs = {
        .on_reach = _pcnt_on_reach_cb,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(_pcnt_unit, &cbs, this));

    ESP_ERROR_CHECK(pcnt_unit_enable(_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(_pcnt_unit));

    _is_initialized = true;
    return ESP_OK;
}

esp_err_t Tb6612Encoder::set_duty_cycle(float duty_cycle_percent) {
    if (!_is_initialized) return ESP_ERR_INVALID_STATE;

    // Clamp duty cycle to valid limits
    if (duty_cycle_percent > 100.0f) duty_cycle_percent = 100.0f;
    if (duty_cycle_percent < -100.0f) duty_cycle_percent = -100.0f;

    // Direction control
    if (duty_cycle_percent > 0.0f) {
        gpio_set_level(static_cast<gpio_num_t>(_in1_gpio), 1);
        gpio_set_level(static_cast<gpio_num_t>(_in2_gpio), 0);
    } else if (duty_cycle_percent < 0.0f) {
        gpio_set_level(static_cast<gpio_num_t>(_in1_gpio), 0);
        gpio_set_level(static_cast<gpio_num_t>(_in2_gpio), 1);
    } else {
        // Coast / Stop (or Brake if user logic prefers IN1=1, IN2=1)
        gpio_set_level(static_cast<gpio_num_t>(_in1_gpio), 0);
        gpio_set_level(static_cast<gpio_num_t>(_in2_gpio), 0);
    }

    float abs_duty = (duty_cycle_percent < 0.0f) ? -duty_cycle_percent : duty_cycle_percent;

    // Calculate comparator ticks dynamically
    uint32_t compare_ticks = static_cast<uint32_t>((abs_duty / 100.0f) * _pwm_period_ticks);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(_cmpr, compare_ticks));

    return ESP_OK;
}

esp_err_t Tb6612Encoder::set_speed_rpm(float rpm) {
    if (!_is_initialized) return ESP_ERR_INVALID_STATE;
    
    // Open-loop linear mapping to duty cycle (Passive Library constraint)
    float duty = (rpm / TB6612_MAX_RPM) * 100.0f;
    return set_duty_cycle(duty);
}

esp_err_t Tb6612Encoder::get_pulse_count(int64_t& out_pulse_count) {
    if (!_is_initialized) return ESP_ERR_INVALID_STATE;
    
    int64_t accum1, accum2;
    int current_count = 0;
    
    // Double-read loop (Optimistic Locking) to prevent race condition 
    do {
        portENTER_CRITICAL(&_spinlock);
        accum1 = _accumulated_pulses;
        portEXIT_CRITICAL(&_spinlock);
        
        ESP_ERROR_CHECK(pcnt_unit_get_count(_pcnt_unit, &current_count));
        
        portENTER_CRITICAL(&_spinlock);
        accum2 = _accumulated_pulses;
        portEXIT_CRITICAL(&_spinlock);
    } while (accum1 != accum2);

    out_pulse_count = accum1 + current_count;
    return ESP_OK;
}

esp_err_t Tb6612Encoder::get_current_rpm(float& out_rpm) {
    if (!_is_initialized) return ESP_ERR_INVALID_STATE;

    int64_t current_pulses = 0;
    get_pulse_count(current_pulses);
    int64_t current_time_us = esp_timer_get_time();
    
    portENTER_CRITICAL(&_spinlock);
    int64_t d_pulses = current_pulses - _last_rpm_calc_pulses; // Signed calculation for reverse rotation
    int64_t d_time_us = current_time_us - _last_rpm_calc_time_us;

    if (d_time_us > 0) {
        // Store state for next delta safely
        _last_rpm_calc_pulses = current_pulses;
        _last_rpm_calc_time_us = current_time_us;
    }
    portEXIT_CRITICAL(&_spinlock);

    if (d_time_us <= 0) {
        out_rpm = 0.0f;
        return ESP_OK;
    }

    // Formula: rpm = (d_pulses / ppr) / (d_time_us / 60,000,000 us)
    out_rpm = static_cast<float>(d_pulses * 60000000LL) / static_cast<float>(_encoder_ppr * d_time_us);

    return ESP_OK;
}
