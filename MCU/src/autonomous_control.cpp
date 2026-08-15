#include "../include/autonomous_control.hpp"
#include <cmath>

MotionOutput AutonomousControl::compute(const StateSnapshot& state, float dt_s, uint32_t loop_counter) {
    // 1. Odometry calculation
    float ppr = state.track_config.encoder_ppr;
    if (ppr < 1.0f) ppr = 341.2f; // Fallback safety
    
    // Calculate delta pulses
    int64_t d_pulse_l = state.encoder_l - _prev_encoder_l;
    int64_t d_pulse_r = state.encoder_r - _prev_encoder_r;
    _prev_encoder_l = state.encoder_l;
    _prev_encoder_r = state.encoder_r;

    // Convert to distance: distance = (d_pulses / PPR) * (2 * PI * radius)
    float d_dist_l = (static_cast<float>(d_pulse_l) / ppr) * (2.0f * static_cast<float>(M_PI) * state.physical_config.wheel_radius_mm);
    float d_dist_r = (static_cast<float>(d_pulse_r) / ppr) * (2.0f * static_cast<float>(M_PI) * state.physical_config.wheel_radius_mm);
    
    _total_displacement_mm += (std::abs(d_dist_l) + std::abs(d_dist_r)) / 2.0f;
    float current_displacement = _total_displacement_mm - _reference_displacement_mm;

    // 2. State Machine execution
    if (loop_counter % PID_EXEC_DECIMATION == 0) {
        float e2 = _line_tracker.compute_e2(state.adc_raw, state.line_calib, state.physical_config);
        
        switch (_current_state) {
            case TrackState::MOVING_TO_PICKUP: {
                // Check if we hit the cross-line (all sensors see black, assuming black line on white surface)
                if (state.adc_raw[0] > 2500 && state.adc_raw[4] > 2500) {
                    _current_state = TrackState::WAITING_FOR_PACKAGE;
                    _last_target_rpm_l = 0.0f;
                    _last_target_rpm_r = 0.0f;
                    break;
                }

                // Dynamic speed selection based on slow zones
                float current_v_ref = state.track_config.v_ref_normal;
                if (current_displacement >= state.track_config.slow_zone_start_mm && 
                    current_displacement <= state.track_config.slow_zone_end_mm) {
                    current_v_ref = state.track_config.v_ref_turn;
                }

                // Override physical_config.v_ref with our dynamic track speed
                RobotPhysicalConfig dyn_config = state.physical_config;
                dyn_config.v_ref = current_v_ref;

                // Normal PID
                _line_tracker.compute_target_rpm(e2, PID_OUTER_DT_S, dyn_config, _last_target_rpm_l, _last_target_rpm_r);
                break;
            }

            case TrackState::WAITING_FOR_PACKAGE: {
                _last_target_rpm_l = 0.0f;
                _last_target_rpm_r = 0.0f;
                
                float weight = state.loadcell_weight;
                if (weight >= state.track_config.loadcell_type1_min && weight <= state.track_config.loadcell_type1_max) {
                    // Type 1 -> Turn Left
                    _current_state = TrackState::DELIVERING_TYPE_1;
                    _reference_displacement_mm = _total_displacement_mm; // Reset tracking
                    _recovery_ticks = 0;
                } 
                else if (weight >= state.track_config.loadcell_type2_min && weight <= state.track_config.loadcell_type2_max) {
                    // Type 2 -> Turn Right
                    _current_state = TrackState::DELIVERING_TYPE_2;
                    _reference_displacement_mm = _total_displacement_mm; // Reset tracking
                    _recovery_ticks = 0;
                }
                break;
            }

            case TrackState::DELIVERING_TYPE_1: {
                // Type 1: Turn Left Hard
                _last_target_rpm_l = state.track_config.turn_phase1_inner_rpm; 
                _last_target_rpm_r = state.track_config.turn_phase1_outer_rpm; 
                _recovery_ticks++;
                
                if (_recovery_ticks > state.track_config.turn_phase1_timeout_ticks || state.adc_raw[3] > 2000) {
                    _current_state = TrackState::RECOVERY_PHASE_2;
                    _recovery_ticks = 0;
                }
                break;
            }

            case TrackState::DELIVERING_TYPE_2: {
                // Type 2: Turn Right Hard
                _last_target_rpm_l = state.track_config.turn_phase1_outer_rpm; 
                _last_target_rpm_r = state.track_config.turn_phase1_inner_rpm; 
                _recovery_ticks++;
                
                if (_recovery_ticks > state.track_config.turn_phase1_timeout_ticks || state.adc_raw[1] > 2000) {
                    _current_state = TrackState::RECOVERY_PHASE_2;
                    _recovery_ticks = 0;
                }
                break;
            }

            case TrackState::RECOVERY_PHASE_2: {
                // Pivot until center sensor is back on line
                if (_last_target_rpm_l > _last_target_rpm_r) { // Was turning Right
                    _last_target_rpm_l = state.track_config.turn_phase2_outer_rpm;
                    _last_target_rpm_r = state.track_config.turn_phase2_inner_rpm;
                } else { // Was turning Left
                    _last_target_rpm_l = state.track_config.turn_phase2_inner_rpm;
                    _last_target_rpm_r = state.track_config.turn_phase2_outer_rpm;
                }

                // If center sensor sees line and error is small
                if (state.adc_raw[2] > state.track_config.turn_phase2_center_threshold && std::abs(e2) < 8.0f) {
                    _current_state = TrackState::MOVING_TO_PICKUP; 
                    _line_tracker.reset();
                }
                break;
            }

            case TrackState::FINISHED:
                _last_target_rpm_l = 0.0f;
                _last_target_rpm_r = 0.0f;
                break;
        }
    }

    MotionOutput out;
    out.target_rpm_left = _last_target_rpm_l;
    out.target_rpm_right = _last_target_rpm_r;

    return out;
}

void AutonomousControl::reset() {
    _current_state = TrackState::MOVING_TO_PICKUP;
    _total_displacement_mm = 0.0f;
    _reference_displacement_mm = 0.0f;
    _recovery_ticks = 0;
    _last_target_rpm_l = 0.0f;
    _last_target_rpm_r = 0.0f;
    _line_tracker.reset();
    
    // We don't reset _prev_encoder_l/r here because they should track 
    // the continuous absolute encoder values. The next compute() will
    // just diff them from the current encoder values. Wait, if we reset 
    // while moving, the diff might be large? No, compute is called at 100Hz.
    // Actually, it's safer to just set _total_displacement_mm = 0.
}
