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

    float e2 = _line_tracker.compute_e2(masked_adc, state.line_calib,
                                        state.physical_config);
    _last_e2 = e2;

    switch (_current_state) {
    case TrackState::MOVING_TO_PICKUP: {
      // 1. End of line detection (5 sensors white) OR Line lost recovery
      if (state.adc_raw[0] < 500 && state.adc_raw[1] < 500 &&
          state.adc_raw[2] < 500 && state.adc_raw[3] < 500 &&
          state.adc_raw[4] < 500) {
        
        if (state.encoder_l >= 33000 && state.encoder_r >= 33000) {
          // Reached the end of the track
          _current_state = TrackState::FINISHED;
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

      // 1.5. Fallback stop
      if (!_is_carrying_package && state.encoder_l > 10000 &&
          state.encoder_r > 10000) {
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
          if (_recovery_ticks >= 50) { // 0.5s continuous invalid load
            _current_state = TrackState::FINISHED; // Stop permanently
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

      // Use physical config's v_ref (which is updated via UDP 'tune' and saved
      // to NVS)
      RobotPhysicalConfig dyn_config = state.physical_config;

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
        _current_state = TrackState::MOVING_TO_PICKUP;
        _recovery_ticks = 0;
        _reference_displacement_mm =
            _total_displacement_mm; // Reset odometry for Y-junction filter
        _line_tracker.reset();
      }
      break;
    }

    case TrackState::DELIVERING_TYPE_1: {
      // Type 1: Turn Left using Sensor Masking & PID
      RobotPhysicalConfig dyn_config = state.physical_config;
      dyn_config.v_ref = state.track_config.v_ref_turn; // Slower speed for turning

      _line_tracker.compute_target_rpm(e2, PID_OUTER_DT_S, dyn_config,
                                       _last_target_rpm_l, _last_target_rpm_r);
      _recovery_ticks++;

      if (_recovery_ticks >= state.track_config.turn_phase1_timeout_ticks) {
        _current_state = TrackState::MOVING_TO_PICKUP;
        _recovery_ticks = 0;
        // Do not reset line tracker here to allow smooth derivative transition
      }
      break;
    }

    case TrackState::DELIVERING_TYPE_2: {
      // Type 2: Turn Right using Sensor Masking & PID
      RobotPhysicalConfig dyn_config = state.physical_config;
      dyn_config.v_ref = state.track_config.v_ref_turn; // Slower speed for turning

      _line_tracker.compute_target_rpm(e2, PID_OUTER_DT_S, dyn_config,
                                       _last_target_rpm_l, _last_target_rpm_r);
      _recovery_ticks++;

      if (_recovery_ticks >= state.track_config.turn_phase1_timeout_ticks) {
        _current_state = TrackState::MOVING_TO_PICKUP;
        _recovery_ticks = 0;
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

  // Reset encoder reference to 0
  _prev_encoder_l = 0;
  _prev_encoder_r = 0;
}
