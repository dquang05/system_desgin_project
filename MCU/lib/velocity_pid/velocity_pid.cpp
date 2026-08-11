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
    _prev_error = 0.0f;
    _prev_prev_error = 0.0f;
    _current_setpoint = 0.0f;
    _prev_output = 0.0f;
}

float VelocityPid::compute(float current_vel, float dt_s) {
    if (dt_s <= 0.0f) {
        return _prev_output;
    }

    // Setpoint Ramping
    float max_step = _config.max_accel_units_s2 * dt_s;
    float setpoint_diff = _target_setpoint - _current_setpoint;
    float step = std::clamp(setpoint_diff, -max_step, max_step);
    _current_setpoint += step;

    // Deadband for absolute zero target to stop "hunting"
    if (std::abs(_target_setpoint) < 0.1f && std::abs(_current_setpoint) < 0.1f) {
        reset();
        return 0.0f;
    }

    float error = _current_setpoint - current_vel;

    // Incremental PID Formula: 
    // delta_output = Kp * (error - prev_error) + Ki * error * dt + Kd * (error - 2*prev_error + prev_prev_error) / dt
    float delta_p = _config.kp * (error - _prev_error);
    float delta_i = _config.ki * error * dt_s;
    float delta_d = _config.kd * (error - 2.0f * _prev_error + _prev_prev_error) / dt_s;

    float delta_output = delta_p + delta_i + delta_d;
    float total_output = _prev_output + delta_output;

    // Clamp the final output to prevent wind-up and exceed hardware limits
    total_output = std::clamp(total_output, _config.out_min, _config.out_max);

    // Save states for next cycle
    _prev_prev_error = _prev_error;
    _prev_error = error;
    _prev_output = total_output;

    return total_output;
}

// RISK REVIEW:
// - Mathematical Risk (Jitter): If the calling interval `dt_s` fluctuates significantly (Jitter), the derivative term (D) will be noisy and unstable, potentially causing spikes in the output. The slew rate limiter step size will also vary, making the acceleration profile inconsistent.
// - Caller Responsibility: The caller MUST explicitly call `reset()` when transitioning the robot from Disable to Enable state (or recovering from an error). Failure to do so will result in Integral Windup due to stale accumulated errors and sudden jerks caused by an unreset `_current_setpoint`.
