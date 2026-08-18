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
                // Continuous debounce for package removal (must be < 200g for 50 consecutive ticks = 500ms)
                if (_is_carrying_package) {
                    if (state.loadcell_weight < 200.0f) {
                        _recovery_ticks++;
                        if (_recovery_ticks > 50) {
                            _is_carrying_package = false;
                            _cargo_type = 0;
                            _recovery_ticks = 0;
                        }
                    } else {
                        _recovery_ticks = 0; // Reset timer if weight bounces back up
                    }
                }

                // Cross-line detection
                if (state.adc_raw[1] > 2500 && state.adc_raw[2] > 2500 && state.adc_raw[3] > 2500 && state.adc_raw[0] < 2000 && state.adc_raw[4] < 2000) {
                    if (!_is_carrying_package) {
                        // Not carrying package -> Stop at pickup point
                        _current_state = TrackState::WAITING_FOR_PACKAGE;
                        _last_target_rpm_l = 0.0f;
                        _last_target_rpm_r = 0.0f;
                        _line_tracker.reset();
                        break;
                    } else if (current_displacement > 200.0f) {
                        // Carrying package AND hit a cross-line (T-junction!)
                        // Transition to differential steering based on cargo type
                        if (_cargo_type == 1) {
                            _current_state = TrackState::DELIVERING_TYPE_1;
                        } else if (_cargo_type == 2) {
                            _current_state = TrackState::DELIVERING_TYPE_2;
                        }
                        _recovery_ticks = 0;
                        break;
                    }
                }

                // Use normal reference speed (Slow zone removed)
                RobotPhysicalConfig dyn_config = state.physical_config;
                dyn_config.v_ref = state.track_config.v_ref_normal;

                // Normal PID
                _line_tracker.compute_target_rpm(e2, PID_OUTER_DT_S, dyn_config, _last_target_rpm_l, _last_target_rpm_r);
                break;
            }

            case TrackState::WAITING_FOR_PACKAGE: {
                _last_target_rpm_l = 0.0f;
                _last_target_rpm_r = 0.0f;
                
                float weight = state.loadcell_weight;
                // If any cargo > 500g is loaded, save type and prepare to start
                if (weight > 500.0f) {
                    if (weight < 1500.0f) {
                        _cargo_type = 1; // ~1kg -> Left
                    } else {
                        _cargo_type = 2; // ~2kg -> Right
                    }
                    _current_state = TrackState::DELAY_BEFORE_START;
                    _recovery_ticks = 0;
                }
                break;
            }

            case TrackState::DELAY_BEFORE_START: {
                _last_target_rpm_l = 0.0f;
                _last_target_rpm_r = 0.0f;
                _recovery_ticks++;
                
                // Wait 20 ticks (1000ms) before starting
                if (_recovery_ticks >= 20) {
                    _is_carrying_package = true; // Set the cargo flag
                    _current_state = TrackState::MOVING_TO_PICKUP; // Resume moving straight
                    _recovery_ticks = 0;
                    _reference_displacement_mm = _total_displacement_mm; // Reset tracking!
                    _line_tracker.reset();
                }
                break;
            }

            case TrackState::DELIVERING_TYPE_1: {
                // Type 1: Turn Left Smoothly (Differential Steering)
                _last_target_rpm_l = state.track_config.turn_phase1_inner_rpm; 
                _last_target_rpm_r = state.track_config.turn_phase1_outer_rpm; 
                _recovery_ticks++;
                
                // Maintain differential steering for 10 ticks (500ms) to guide into the curve
                if (_recovery_ticks >= 10) {
                    _current_state = TrackState::MOVING_TO_PICKUP;
                    _recovery_ticks = 0;
                    _line_tracker.reset();
                }
                break;
            }

            case TrackState::DELIVERING_TYPE_2: {
                // Type 2: Turn Right Smoothly (Differential Steering)
                _last_target_rpm_l = state.track_config.turn_phase1_outer_rpm; 
                _last_target_rpm_r = state.track_config.turn_phase1_inner_rpm; 
                _recovery_ticks++;
                
                // Maintain differential steering for 10 ticks (500ms) to guide into the curve
                if (_recovery_ticks >= 10) {
                    _current_state = TrackState::MOVING_TO_PICKUP;
                    _recovery_ticks = 0;
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
    _is_carrying_package = false;
    _cargo_type = 0;
    _line_tracker.reset();
    
    // We don't reset _prev_encoder_l/r here because they should track 
    // the continuous absolute encoder values. The next compute() will
    // just diff them from the current encoder values. Wait, if we reset 
    // while moving, the diff might be large? No, compute is called at 100Hz.
    // Actually, it's safer to just set _total_displacement_mm = 0.
}
