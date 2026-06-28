#pragma once

#include <cmath>
#include <algorithm>

/**
 * @brief Configuration structure for Velocity PID.
 */
struct velocity_pid_config_t {
    float kp;
    float ki;
    float kd;
    float out_max;
    float out_min;
    float integral_max;
    float max_accel_units_s2;
};

/**
 * @brief Velocity PID controller class.
 * 
 * This class implements a PID controller specifically designed for velocity
 * control. It includes setpoint ramping (slew rate limiter) and integral
 * anti-windup using the clamping method. It is purely mathematical and
 * decoupled from hardware.
 */
class VelocityPid {
public:
    /**
     * @brief Construct a new Velocity Pid object.
     */
    VelocityPid() = default;

    /**
     * @brief Initialize the PID controller with the given configuration.
     * 
     * @param config The configuration structure containing PID parameters and limits.
     */
    void init(const velocity_pid_config_t& config);

    /**
     * @brief Update PID tunings dynamically.
     * 
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     */
    void set_tunings(float kp, float ki, float kd);

    /**
     * @brief Set the target velocity. The internal setpoint will ramp towards this value.
     * 
     * @param target_vel The desired target velocity.
     */
    void set_target_velocity(float target_vel);

    /**
     * @brief Reset the internal state of the PID controller (integral sum, previous error, current setpoint).
     */
    void reset();

    /**
     * @brief Compute the new control output based on the current velocity and time step.
     * 
     * @param current_vel The current measured velocity.
     * @param dt_s The time elapsed since the last computation in seconds.
     * @return float The control variable (output) limited by [out_min, out_max].
     */
    float compute(float current_vel, float dt_s);

private:
    velocity_pid_config_t _config{};
    
    float _target_setpoint{0.0f};
    float _current_setpoint{0.0f};
    
    float _integral_sum{0.0f};
    float _prev_error{0.0f};
    float _prev_output{0.0f};
};
