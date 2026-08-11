#include "../include/autonomous_control.hpp"

MotionOutput AutonomousControl::compute(const StateSnapshot& state, float dt_s, uint32_t loop_counter) {
    // Execute Line Tracking PID every PID_EXEC_DECIMATION ticks (e.g. 50ms / 20Hz)
    if (loop_counter % PID_EXEC_DECIMATION == 0) {
        float e2 = _line_tracker.compute_e2(state.adc_raw, state.line_calib, state.physical_config);
        _line_tracker.compute_target_rpm(e2, PID_OUTER_DT_S, state.physical_config, _last_target_rpm_l, _last_target_rpm_r);
    }

    MotionOutput out;
    out.target_rpm_left = _last_target_rpm_l;
    out.target_rpm_right = _last_target_rpm_r;

    return out;
}
