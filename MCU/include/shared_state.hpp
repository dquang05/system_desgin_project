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
    float loadcell_weight;
    LineSensorCalib line_calib;
    RobotPhysicalConfig physical_config;
};
