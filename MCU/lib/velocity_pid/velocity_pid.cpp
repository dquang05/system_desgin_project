#include "velocity_pid.hpp"

void VelocityPid::init(const velocity_pid_config_t& config) {
    _config = config;
    reset();
}

void VelocityPid::set_tunings(float kp, float ki, float kd) {
    _config.kp = kp;
    _config.ki = ki;
    _config.kd = kd;
}

void VelocityPid::set_target_velocity(float target_vel) {
    _target_setpoint = target_vel;
}

void VelocityPid::reset() {
    _integral_sum = 0.0f;
    _prev_error = 0.0f;
    _current_setpoint = 0.0f;
    _prev_output = 0.0f;
}

float VelocityPid::compute(float current_vel, float dt_s) {
    // Edge case: invalid dt_s to prevent division by zero or negative time
    if (dt_s <= 0.0f) {
        return _prev_output;
    }

    // Setpoint Ramping (Slew Rate Limiter)
    float max_step = _config.max_accel_units_s2 * dt_s;
    float setpoint_diff = _target_setpoint - _current_setpoint;
    
    // Clamp the step step to [-max_step, max_step]
    float step = std::clamp(setpoint_diff, -max_step, max_step);
    _current_setpoint += step;

    // Calculate error based on the ramped setpoint
    float error = _current_setpoint - current_vel;

    // Calculate Proportional term
    float p_term = _config.kp * error;

    // Calculate Derivative term
    float d_term = _config.kd * (error - _prev_error) / dt_s;

    // Calculate total provisional output (P + provisional I + D)
    float provisional_i_term = _config.ki * (_integral_sum + error * dt_s);
    float provisional_output = p_term + provisional_i_term + d_term;

    // Integral Anti-Windup (Clamping Method)
    bool is_saturated = (provisional_output > _config.out_max) || (provisional_output < _config.out_min);
    bool is_same_sign = (error * provisional_output > 0.0f);

    if (is_saturated && is_same_sign) {
        // Output is saturated and the error is pushing it further into saturation.
        // Stop integrating the error.
    } else {
        // Safe to accumulate
        _integral_sum += error * dt_s;
    }

    // Clamp the integral sum to its limits
    _integral_sum = std::clamp(_integral_sum, -_config.integral_max, _config.integral_max);

    // Calculate final total output with the updated and clamped integral sum
    float i_term = _config.ki * _integral_sum;
    float total_output = p_term + i_term + d_term;

    // Clamp the final output
    total_output = std::clamp(total_output, _config.out_min, _config.out_max);

    // Save states for next cycle
    _prev_error = error;
    _prev_output = total_output;

    return total_output;
}

// RISK REVIEW:
// - Mathematical Risk (Jitter): If the calling interval `dt_s` fluctuates significantly (Jitter), the derivative term (D) will be noisy and unstable, potentially causing spikes in the output. The slew rate limiter step size will also vary, making the acceleration profile inconsistent.
// - Caller Responsibility: The caller MUST explicitly call `reset()` when transitioning the robot from Disable to Enable state (or recovering from an error). Failure to do so will result in Integral Windup due to stale accumulated errors and sudden jerks caused by an unreset `_current_setpoint`.
