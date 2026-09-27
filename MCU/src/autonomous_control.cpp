#include "../include/autonomous_control.hpp"
#include <cmath>

MotionOutput AutonomousControl::compute(const StateSnapshot &state, float dt_s,
                                        uint32_t loop_counter) {
  // 1. Odometry calculation
  float ppr = state.track_config.encoder_ppr;
  if (ppr < 1.0f)
    ppr = 341.2f; // Fallback safety

  // Calculate delta pulses
  int64_t d_pulse_l = state.encoder_l - _prev_encoder_l;
  int64_t d_pulse_r = state.encoder_r - _prev_encoder_r;
  _prev_encoder_l = state.encoder_l;
  _prev_encoder_r = state.encoder_r;

  // Convert to distance: distance = (d_pulses / PPR) * (2 * PI * radius)
  float d_dist_l =
      (static_cast<float>(d_pulse_l) / ppr) *
      (2.0f * static_cast<float>(M_PI) * state.physical_config.wheel_radius_mm);
  float d_dist_r =
      (static_cast<float>(d_pulse_r) / ppr) *
      (2.0f * static_cast<float>(M_PI) * state.physical_config.wheel_radius_mm);

  _total_displacement_mm += (std::abs(d_dist_l) + std::abs(d_dist_r)) / 2.0f;
  float current_displacement =
      _total_displacement_mm - _reference_displacement_mm;

  // 2. State Machine execution
  if (loop_counter % PID_EXEC_DECIMATION == 0) {
    uint32_t masked_adc[ROBOT_NUM_SENSORS];
    for (int i = 0; i < ROBOT_NUM_SENSORS; i++) {
      masked_adc[i] = state.adc_raw[i];
    }

    if (_current_state == TrackState::DELIVERING_TYPE_1) {
      masked_adc[3] = 0;
      masked_adc[4] = 0;
    } else if (_current_state == TrackState::DELIVERING_TYPE_2) {
      masked_adc[0] = 0;
      masked_adc[1] = 0;
    }

    bool in_curve = false;
    if (_current_state == TrackState::MOVING_TO_PICKUP && _is_carrying_package) {
      int64_t d_pulse_pickup_l = state.encoder_l - _post_pickup_enc_l;
      int64_t d_pulse_pickup_r = state.encoder_r - _post_pickup_enc_r;
      in_curve =
          (d_pulse_pickup_l >= 2700 && (d_pulse_pickup_l - 2700) < 10000) ||
          (d_pulse_pickup_r >= 2700 && (d_pulse_pickup_r - 2700) < 7000);
    }

    float e2 = _line_tracker.compute_e2(masked_adc, state.line_calib,
                                        state.physical_config);
    _last_e2 = e2;

    switch (_current_state) {
    case TrackState::MOVING_TO_PICKUP: {
      // 1. End of line detection (5 sensors white) OR Line lost recovery
      if (state.adc_raw[0] < 2500 && state.adc_raw[1] < 2500 &&
          state.adc_raw[2] < 2500 && state.adc_raw[3] < 2500 &&
          state.adc_raw[4] < 2500) {

        int64_t post_turn_l = state.encoder_l - _post_turn_encoder_l;
        int64_t post_turn_r = state.encoder_r - _post_turn_encoder_r;
        int64_t finish_thresh = state.track_config.finish_stop_pulses > 0
                                    ? state.track_config.finish_stop_pulses
                                    : 10000;
        if (_has_turned && post_turn_l >= finish_thresh &&
            post_turn_r >= finish_thresh) {
          // Reached the end of the track
          _current_state = TrackState::FINISHED;
          _last_target_rpm_l = 0.0f;
          _last_target_rpm_r = 0.0f;
          break;
        } else {
          // Lost line, perform recovery based on last known error (e2)
          if (e2 > 0.0f) {
            // Line was to the right, car veered left. Steer Right.
            _last_target_rpm_l = state.track_config.turn_phase1_outer_rpm;
            _last_target_rpm_r = state.track_config.turn_phase1_inner_rpm;
          } else {
            // Line was to the left, car veered right. Steer Left.
            _last_target_rpm_l = state.track_config.turn_phase1_inner_rpm;
            _last_target_rpm_r = state.track_config.turn_phase1_outer_rpm;
          }
        }
        break; // Skip normal PID
      }

      // 1.5. Fallback stop at pickup point
      int64_t pickup_stop = state.track_config.pickup_stop_pulses > 0
                                ? state.track_config.pickup_stop_pulses
                                : 10800;
      if (!_is_carrying_package && state.encoder_l >= pickup_stop &&
          state.encoder_r >= pickup_stop) {
        _current_state = TrackState::WAITING_FOR_PACKAGE;
        _last_target_rpm_l = 0.0f;
        _last_target_rpm_r = 0.0f;
        _line_tracker.reset();
        break;
      }

      // 2. Cargo validation if carrying
      if (_is_carrying_package) {
        float weight = state.loadcell_weight;
        bool weight_valid = false;

        if (_cargo_type == 1 &&
            weight >= state.track_config.loadcell_type1_min &&
            weight <= state.track_config.loadcell_type1_max) {
          weight_valid = true;
        } else if (_cargo_type == 2 &&
                   weight >= state.track_config.loadcell_type2_min &&
                   weight <= state.track_config.loadcell_type2_max) {
          weight_valid = true;
        }

        if (!weight_valid) {
          _recovery_ticks++;
          if (_recovery_ticks >= 6) { // ~300ms continuous invalid load (filter vibration)
            _current_state = TrackState::FINISHED; // Stop permanently
            _last_target_rpm_l = 0.0f;
            _last_target_rpm_r = 0.0f;
            break;
          }
        } else {
          _recovery_ticks = 0; // Reset debounce if load bounces back
        }
      }

      // 3. T-Junction (Pickup point): Center 3 black, outer 2 white
      if (!_is_carrying_package && state.adc_raw[0] < 2000 &&
          state.adc_raw[1] > 2000 && state.adc_raw[2] > 2000 &&
          state.adc_raw[3] > 2000 && state.adc_raw[4] < 2000) {
        _current_state = TrackState::WAITING_FOR_PACKAGE;
        _last_target_rpm_l = 0.0f;
        _last_target_rpm_r = 0.0f;
        _line_tracker.reset();
        break;
      }

      // 4. Y-Junction (Turn point): Center 3 black, outer 2 white
      if (_is_carrying_package && current_displacement > 200.0f) {
        if (state.adc_raw[0] < 2000 && state.adc_raw[1] > 2000 &&
            state.adc_raw[2] > 2000 && state.adc_raw[3] > 2000 &&
            state.adc_raw[4] < 2000) {
          if (_cargo_type == 1) {
            _current_state = TrackState::DELIVERING_TYPE_1;
          } else if (_cargo_type == 2) {
            _current_state = TrackState::DELIVERING_TYPE_2;
          }
          _recovery_ticks = 0;
          break;
        }
      }

      // Use physical config (updated via UDP 'tune' and saved to NVS)
      RobotPhysicalConfig dyn_config = state.physical_config;

      // Default: Use Set 1 (kp, kd, pid_tau) for all normal segments
      dyn_config.kp = state.physical_config.kp;
      dyn_config.kd = state.physical_config.kd;
      dyn_config.pid_tau = state.physical_config.pid_tau;
      dyn_config.v_ref = state.physical_config.v_ref;

      if (_is_carrying_package) {
        int64_t d_pulse_pickup_l = state.encoder_l - _post_pickup_enc_l;
        int64_t d_pulse_pickup_r = state.encoder_r - _post_pickup_enc_r;

        bool after_curve =
            !in_curve && (d_pulse_pickup_l >= 2700 || d_pulse_pickup_r >= 2700);

        if (in_curve) {
          // Inside the R=500mm curve: Use Set 2 for BOTH cargo types, with
          // v_ref_turn
          dyn_config.kp = state.physical_config.kp_load1;
          dyn_config.kd = state.physical_config.kd_load1;
          dyn_config.pid_tau = state.physical_config.pid_tau_load1;
          dyn_config.v_ref = state.physical_config.v_ref_turn;

          // Virtual center bias: shift sensor center 4.0mm rightward to hug R=500mm right curve
          e2 += 4.0f;
        } else if (after_curve) {
          // After the R=500mm curve: Use Set 3 (formerly Set 1) to finish
          dyn_config.kp = state.physical_config.kp_load2;
          dyn_config.kd = state.physical_config.kd_load2;
          dyn_config.pid_tau = state.physical_config.pid_tau_load2;
          dyn_config.v_ref = state.physical_config.v_ref;

          // Deceleration at 80% of finish distance (~9600 pulses post-turn)
          if (_has_turned) {
            int64_t post_turn_pulses =
                ((state.encoder_l - _post_turn_encoder_l) +
                 (state.encoder_r - _post_turn_encoder_r)) /
                2;
            if (post_turn_pulses >= 12500) {
              dyn_config.v_ref *= 0.20f;
            } else if (post_turn_pulses >= 11000) {
              dyn_config.v_ref *= 0.30f;
            } else if (post_turn_pulses >= 10500) {
              dyn_config.v_ref *= 0.60f;
            }
          }
        } else {
          // Before the curve (the 2700-pulse buffer post-pickup): Use Set 1 &
          // v_ref
          dyn_config.kp = state.physical_config.kp;
          dyn_config.kd = state.physical_config.kd;
          dyn_config.pid_tau = state.physical_config.pid_tau;
          dyn_config.v_ref = state.physical_config.v_ref;
        }
      } else {
        // Not carrying package: Approach pickup station with Set 1
        dyn_config.v_ref = state.physical_config.v_ref;

        // Deceleration at 80% of pickup distance (~8640 pulses from start)
        int64_t avg_pulses = (state.encoder_l + state.encoder_r) / 2;
        int64_t pickup_decel = state.track_config.pickup_decel_pulses > 0
                                   ? state.track_config.pickup_decel_pulses
                                   : 10000;
        if (avg_pulses >= pickup_decel) {
          float decel = (state.track_config.decel_ratio > 0.1f &&
                         state.track_config.decel_ratio < 1.0f)
                            ? state.track_config.decel_ratio
                            : 0.65f;
          dyn_config.v_ref *= decel;
        }
      }

      // Normal PID
      _line_tracker.compute_target_rpm(e2, PID_OUTER_DT_S, dyn_config,
                                       _last_target_rpm_l, _last_target_rpm_r);
      break;
    }

    case TrackState::WAITING_FOR_PACKAGE: {
      _last_target_rpm_l = 0.0f;
      _last_target_rpm_r = 0.0f;

      float weight = state.loadcell_weight;
      if (weight >= state.track_config.loadcell_type1_min &&
          weight <= state.track_config.loadcell_type1_max) {
        _cargo_type = 1; // 1kg -> Left
        _current_state = TrackState::DELAY_BEFORE_START;
        _recovery_ticks = 0;
      } else if (weight >= state.track_config.loadcell_type2_min &&
                 weight <= state.track_config.loadcell_type2_max) {
        _cargo_type = 2; // 2kg -> Right
        _current_state = TrackState::DELAY_BEFORE_START;
        _recovery_ticks = 0;
      }
      break;
    }

    case TrackState::DELAY_BEFORE_START: {
      _last_target_rpm_l = 0.0f;
      _last_target_rpm_r = 0.0f;
      _recovery_ticks++;

      // Check if weight is still valid during the delay
      float weight = state.loadcell_weight;
      bool weight_valid = false;
      if (_cargo_type == 1 && weight >= state.track_config.loadcell_type1_min &&
          weight <= state.track_config.loadcell_type1_max) {
        weight_valid = true;
      } else if (_cargo_type == 2 &&
                 weight >= state.track_config.loadcell_type2_min &&
                 weight <= state.track_config.loadcell_type2_max) {
        weight_valid = true;
      }

      if (!weight_valid) {
        _current_state =
            TrackState::WAITING_FOR_PACKAGE; // Interrupted, wait again
        break;
      }

      // Wait 50 ticks (500ms) before starting
      if (_recovery_ticks >= 50) {
        _is_carrying_package = true;
        _recovery_ticks = 0;
        _reference_displacement_mm =
            _total_displacement_mm; // Reset odometry for Y-junction filter
        _post_pickup_enc_l = state.encoder_l;
        _post_pickup_enc_r = state.encoder_r;
        _line_tracker.reset();

        if (state.track_config.fuzzy_mode) {
          // Switch to Blind Run Stage 1 (Straight)
          _current_state = TrackState::BLIND_RUN_STRAIGHT_1;
          _blind_start_enc_l = state.encoder_l;
          _blind_start_enc_r = state.encoder_r;
          _last_target_rpm_l = state.track_config.blind_seg1_rpm_l;
          _last_target_rpm_r = state.track_config.blind_seg1_rpm_r;
        } else {
          _current_state = TrackState::MOVING_TO_PICKUP;
        }
      }
      break;
    }

    case TrackState::BLIND_RUN_STRAIGHT_1: {
      _last_target_rpm_l = state.track_config.blind_seg1_rpm_l;
      _last_target_rpm_r = state.track_config.blind_seg1_rpm_r;

      int64_t diff_l = std::abs(state.encoder_l - _blind_start_enc_l);
      int64_t diff_r = std::abs(state.encoder_r - _blind_start_enc_r);

      if (diff_l >= state.track_config.blind_seg1_pulses_l &&
          diff_r >= state.track_config.blind_seg1_pulses_r) {
        _current_state = TrackState::BLIND_RUN_CURVE;
        _blind_start_enc_l = state.encoder_l;
        _blind_start_enc_r = state.encoder_r;
        _last_target_rpm_l = state.track_config.blind_seg2_rpm_l;
        _last_target_rpm_r = state.track_config.blind_seg2_rpm_r;
      }
      break;
    }

    case TrackState::BLIND_RUN_CURVE: {
      _last_target_rpm_l = state.track_config.blind_seg2_rpm_l;
      _last_target_rpm_r = state.track_config.blind_seg2_rpm_r;

      int64_t diff_l = std::abs(state.encoder_l - _blind_start_enc_l);
      int64_t diff_r = std::abs(state.encoder_r - _blind_start_enc_r);

      if (diff_l >= state.track_config.blind_seg2_pulses_l ||
          diff_r >= state.track_config.blind_seg2_pulses_r) {
        _current_state = TrackState::BLIND_RUN_STRAIGHT_2;
        _blind_start_enc_l = state.encoder_l;
        _blind_start_enc_r = state.encoder_r;
        _last_target_rpm_l = state.track_config.blind_seg3_rpm_l;
        _last_target_rpm_r = state.track_config.blind_seg3_rpm_r;
      }
      break;
    }

    case TrackState::BLIND_RUN_STRAIGHT_2: {
      _last_target_rpm_l = state.track_config.blind_seg3_rpm_l;
      _last_target_rpm_r = state.track_config.blind_seg3_rpm_r;

      // Condition to transition: Detect turn / Y-junction on ADC
      // Center sensors detect black line while turning area is entered
      if (state.adc_raw[1] > 2000 || state.adc_raw[2] > 2000 ||
          state.adc_raw[3] > 2000) {
        if (_cargo_type == 1) {
          _current_state = TrackState::DELIVERING_TYPE_1;
        } else {
          _current_state = TrackState::DELIVERING_TYPE_2;
        }
        _recovery_ticks = 0;
        _line_tracker.reset();
      }
      break;
    }

    case TrackState::DELIVERING_TYPE_1: {
      // Type 1: Turn Left using Sensor Masking & PID with Set 1 parameters
      RobotPhysicalConfig dyn_config = state.physical_config;
      dyn_config.v_ref = state.physical_config.v_ref;
      dyn_config.kp = state.physical_config.kp;
      dyn_config.kd = state.physical_config.kd;
      dyn_config.pid_tau = state.physical_config.pid_tau;

      _line_tracker.compute_target_rpm(e2, PID_OUTER_DT_S, dyn_config,
                                       _last_target_rpm_l, _last_target_rpm_r);
      _recovery_ticks++;

      if (_recovery_ticks >= state.track_config.turn_phase1_timeout_ticks) {
        _current_state = TrackState::MOVING_TO_PICKUP;
        _recovery_ticks = 0;
        _has_turned = true;
        _post_turn_encoder_l = state.encoder_l;
        _post_turn_encoder_r = state.encoder_r;
        // Do not reset line tracker here to allow smooth derivative transition
      }
      break;
    }

    case TrackState::DELIVERING_TYPE_2: {
      // Type 2: Turn Right using Sensor Masking & PID with Set 1 parameters
      RobotPhysicalConfig dyn_config = state.physical_config;
      dyn_config.v_ref = state.physical_config.v_ref;
      dyn_config.kp = state.physical_config.kp;
      dyn_config.kd = state.physical_config.kd;
      dyn_config.pid_tau = state.physical_config.pid_tau;

      _line_tracker.compute_target_rpm(e2, PID_OUTER_DT_S, dyn_config,
                                       _last_target_rpm_l, _last_target_rpm_r);
      _recovery_ticks++;

      if (_recovery_ticks >= state.track_config.turn_phase1_timeout_ticks) {
        _current_state = TrackState::MOVING_TO_PICKUP;
        _recovery_ticks = 0;
        _has_turned = true;
        _post_turn_encoder_l = state.encoder_l;
        _post_turn_encoder_r = state.encoder_r;
        // Do not reset line tracker here to allow smooth derivative transition
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
  out.current_e2 = _last_e2;
  out.is_finished = (_current_state == TrackState::FINISHED);
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
  _post_pickup_enc_l = 0;
  _post_pickup_enc_r = 0;
  _has_turned = false;
  _post_turn_encoder_l = 0;
  _post_turn_encoder_r = 0;
  _blind_start_enc_l = 0;
  _blind_start_enc_r = 0;
  _line_tracker.reset();

  // Reset encoder reference to 0
  _prev_encoder_l = 0;
  _prev_encoder_r = 0;
}
