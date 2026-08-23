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
    float kp;      // Line tracker kp
    float kd;      // Line tracker kd
    float pid_tau; // Line tracker tau
    float kp_l;    // Motor left kp
    float ki_l;    // Motor left ki
    float kd_l;    // Motor left kd
    float kp_r;    // Motor right kp
    float ki_r;    // Motor right ki
    float kd_r;    // Motor right kd
};

struct TrackStrategyConfig {
    // Odometry parameters
    float encoder_ppr;           // Pulses per revolution

    // Straight line and corner speeds
    float v_ref_normal;          // Target speed normally
    float v_ref_turn;            // Target speed in slow zone
    float slow_zone_start_mm;    // Start of slow zone
    float slow_zone_end_mm;      // End of slow zone

    // Steering Phase 1 (Hard Turn)
    float turn_phase1_outer_rpm; // RPM for the outer wheel
    float turn_phase1_inner_rpm; // RPM for the inner wheel
    uint32_t turn_phase1_timeout_ticks; // Max ticks (e.g. at 50ms per tick)
    
    // Steering Phase 2 (Pivot/Align)
    float turn_phase2_outer_rpm; // RPM for the outer wheel
    float turn_phase2_inner_rpm; // RPM for the inner wheel
    float turn_phase2_center_threshold; // ADC threshold for center sensor to consider aligned

    // Loadcell thresholds
    float loadcell_type1_min;
    float loadcell_type1_max;
    float loadcell_type2_min;
    float loadcell_type2_max;
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
};
