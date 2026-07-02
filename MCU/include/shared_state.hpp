#pragma once
#include <freertos/FreeRTOS.h>
#include <stdint.h>

#define ROBOT_NUM_SENSORS 5

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
};
