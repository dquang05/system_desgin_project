#pragma once
#include "shared_state.hpp"

struct MotionOutput {
    float target_rpm_left;
    float target_rpm_right;
};

struct StateSnapshot {
    uint32_t adc_raw[ROBOT_NUM_SENSORS];
    LineSensorCalib line_calib;
    RobotPhysicalConfig physical_config;
    float manual_cmd_l;
    float manual_cmd_r;
    float loadcell_weight;
    int64_t encoder_l;
    int64_t encoder_r;
    TrackStrategyConfig track_config;
};

class IMotionStrategy {
public:
    virtual ~IMotionStrategy() = default;
    
    /**
     * @brief Computes the target RPMs for the motors based on the current state.
     * 
     * @param state A thread-safe snapshot of the current shared robot state.
     * @param dt_s The delta time in seconds since the last computation.
     * @param loop_counter The current tick counter of the inner loop (e.g. at 100Hz).
     * @return MotionOutput The target RPMs to be fed into the Velocity PID controllers.
     */
    virtual MotionOutput compute(const StateSnapshot& state, float dt_s, uint32_t loop_counter) = 0;
};
