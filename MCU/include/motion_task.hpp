#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "tb6612_encoder.hpp"
#include "velocity_pid.hpp"

// Define wheel radius in mm for RPM to mm/s conversion
constexpr float WHEEL_RADIUS_MM = 33.0f; // Example: 66mm diameter

struct wheel_vel_t {
    float left;
    float right;
};

// Global Target Velocity
extern wheel_vel_t global_target_vel;

// Global Mutex to protect global_target_vel
extern SemaphoreHandle_t global_vel_mutex;

// Extern declarations for motor and pid objects
extern Tb6612Encoder motor_left;
extern Tb6612Encoder motor_right;
extern VelocityPid pid_left;
extern VelocityPid pid_right;

/**
 * @brief Initialize motion control task resources and start the FreeRTOS task.
 */
void motion_task_start();
