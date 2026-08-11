#include "../include/manual_control.hpp"

MotionOutput ManualControl::compute(const StateSnapshot& state, float dt_s, uint32_t loop_counter) {
    // In manual mode, we directly bypass any autonomous calculations
    // and pipe the manual commands from UDP directly to the output targets.
    
    MotionOutput out;
    out.target_rpm_left = state.manual_cmd_l;
    out.target_rpm_right = state.manual_cmd_r;
    
    return out;
}
