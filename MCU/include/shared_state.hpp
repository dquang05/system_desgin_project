#pragma once
#include <freertos/FreeRTOS.h>
#include <stdint.h>

#define ROBOT_NUM_SENSORS 5

struct LineSensorCalib {
  uint32_t x_max[ROBOT_NUM_SENSORS];
  uint32_t x_min[ROBOT_NUM_SENSORS];
  uint32_t y_max;
  uint32_t y_min;
  float line_coe_1;
  float line_coe_2;
};

struct RobotPhysicalConfig {
  float wheel_base_mm;
  float wheel_radius_mm;
  float sensor_distance_mm;
  float v_ref;
  float v_ref_turn;
  // Line tracker No Load (Default)
  float kp;
  float kd;
  float pid_tau;

  // Line tracker Load Type 1
  float kp_load1;
  float kd_load1;
  float pid_tau_load1;

  // Line tracker Load Type 2
  float kp_load2;
  float kd_load2;
  float pid_tau_load2;

  // Sensor Weights
  float sensor_weight_04;
  float sensor_weight_13;
  float kp_l; // Motor left kp
  float ki_l; // Motor left ki
  float kd_l; // Motor left kd
  float kp_r; // Motor right kp
  float ki_r; // Motor right ki
  float kd_r; // Motor right kd
};

struct TrackStrategyConfig {
  // Odometry parameters
  float encoder_ppr; // Pulses per revolution

  // Straight line and corner speeds

  // Steering Phase 1 (Hard Turn)
  float turn_phase1_outer_rpm;        // RPM for the outer wheel
  float turn_phase1_inner_rpm;        // RPM for the inner wheel
  uint32_t turn_phase1_timeout_ticks; // Max ticks (e.g. at 50ms per tick)

  // Loadcell thresholds
  float loadcell_type1_min;
  float loadcell_type1_max;
  float loadcell_type2_min;
  float loadcell_type2_max;

  // Fuzzy / Blind Run configuration
  bool fuzzy_mode;
  float blind_seg1_rpm_l;
  float blind_seg1_rpm_r;
  int64_t blind_seg1_pulses_l;
  int64_t blind_seg1_pulses_r;

  float blind_seg2_rpm_l;
  float blind_seg2_rpm_r;
  int64_t blind_seg2_pulses_l;
  int64_t blind_seg2_pulses_r;

  float blind_seg3_rpm_l;
  float blind_seg3_rpm_r;
};

struct SharedRobotState {
  portMUX_TYPE spinlock;
  uint32_t adc_raw[ROBOT_NUM_SENSORS];
  int64_t encoder_left;
  int64_t encoder_right;
  float pwm_left;
  float pwm_right;
  float target_rpm_left;
  float target_rpm_right;
  float actual_rpm_left;
  float actual_rpm_right;
  float manual_cmd_l; // Target RPM for left motor in Manual Mode
  float manual_cmd_r; // Target RPM for right motor in Manual Mode
  float current_e2;   // Current cross-track error
  float loadcell_weight;
  LineSensorCalib line_calib;
  RobotPhysicalConfig physical_config;
  TrackStrategyConfig track_config;
  bool system_running;
  bool soft_stop_request;

  // Test mode variables (UDP test PID)
  bool test_mode_active;
  float test_target_rpm_l;
  float test_target_rpm_r;
};
