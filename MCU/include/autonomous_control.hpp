#pragma once

#include "motion_strategy.hpp"
#include "shared_state.hpp"
#include "line_tracker.hpp"

/**
 * @brief Represents the high-level mission state of the robot.
 */
enum class TrackState {
    MOVING_TO_PICKUP,    // Normal line tracking until cross-line is detected
    WAITING_FOR_PACKAGE, // Stopped, reading loadcell
    DELAY_BEFORE_START,  // Wait 1s after package is loaded
    DELIVERING_TYPE_1,   // Package 1kg -> Turn Left Smoothly
    DELIVERING_TYPE_2,   // Package 2kg -> Turn Right Smoothly
    FINISHED
};

/**
 * @brief High-level autonomous controller.
 * 
 * Manages the state machine, trajectory logic, loadcell integration,
 * and delegates low-level PID tracking to the LineTracker class.
 */
class AutonomousControl : public IMotionStrategy {
public:
    AutonomousControl() = default;
    ~AutonomousControl() override = default;

    /**
     * @brief Computes the target RPMs for autonomous operation.
     */
    MotionOutput compute(const StateSnapshot& state, float dt_s, uint32_t loop_counter) override;

    /**
     * @brief Resets the state machine and odometry for a new run.
     */
    void reset() override;

private:
    LineTracker _line_tracker;

    // State Machine variables
    TrackState _current_state{TrackState::MOVING_TO_PICKUP};
    bool _is_carrying_package{false};
    uint8_t _cargo_type{0}; // 1 = Type 1 (Left), 2 = Type 2 (Right)
    
    // Odometry
    int64_t _prev_encoder_l{0};
    int64_t _prev_encoder_r{0};
    float _total_displacement_mm{0.0f};
    float _reference_displacement_mm{0.0f}; // Added to fix compilation error
    
    // Recovery Phase timers/counters
    uint32_t _recovery_ticks{0};
    float _last_e2{0.0f};

    // Decimation logic for line tracking execution frequency
    static constexpr uint32_t PID_EXEC_DECIMATION = 5; 
    static constexpr float PID_OUTER_DT_S = 0.05f; // 5 * 10ms = 50ms

    // State caching for target RPMs to maintain them between decimations
    float _last_target_rpm_l{0.0f};
    float _last_target_rpm_r{0.0f};
};
