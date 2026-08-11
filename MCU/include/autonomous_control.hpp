#pragma once
#include "motion_strategy.hpp"
#include "line_tracker.hpp"

class AutonomousControl : public IMotionStrategy {
public:
    AutonomousControl() = default;
    ~AutonomousControl() override = default;

    MotionOutput compute(const StateSnapshot& state, float dt_s, uint32_t loop_counter) override;

private:
    LineTracker _line_tracker;

    // Decimation logic for line tracking execution frequency
    static constexpr uint32_t PID_EXEC_DECIMATION = 5; 
    static constexpr float PID_OUTER_DT_S = 0.05f; // 5 * 10ms = 50ms

    // State caching for target RPMs to maintain them between decimations
    float _last_target_rpm_l{0.0f};
    float _last_target_rpm_r{0.0f};
};
