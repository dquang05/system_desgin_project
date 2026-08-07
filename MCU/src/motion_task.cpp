#include "../include/motion_task.hpp"
#include "../include/shared_state.hpp"
#include "../include/line_tracker.hpp"
#include "../lib/tb6612_encoder/tb6612_encoder.hpp"
#include "../lib/velocity_pid/velocity_pid.hpp"
#include <esp_timer.h>

extern Tb6612Encoder motor_left;
extern Tb6612Encoder motor_right;
extern VelocityPid pid_left;
extern VelocityPid pid_right;

void motion_task_routine(void *pvParameters) {
    SharedRobotState* state = static_cast<SharedRobotState*>(pvParameters);
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t freq_ticks = pdMS_TO_TICKS(10); // 100Hz 

    int64_t last_time_us = esp_timer_get_time();
    int64_t last_pulse_l = 0;
    int64_t last_pulse_r = 0;

    motor_left.get_pulse_count(last_pulse_l);
    motor_right.get_pulse_count(last_pulse_r);

    LineTracker line_tracker;
    uint32_t loop_counter = 0;
    float target_rpm_l = 0.0f;
    float target_rpm_r = 0.0f;

    while (true) {
        int64_t current_pulse_l = 0;
        int64_t current_pulse_r = 0;
        motor_left.get_pulse_count(current_pulse_l);
        motor_right.get_pulse_count(current_pulse_r);

        int64_t current_time_us = esp_timer_get_time();
        int64_t delta_time_us = current_time_us - last_time_us;

        if (delta_time_us > 0) {
            float dt_s = static_cast<float>(delta_time_us) / 1000000.0f;
            float raw_rpm_l = 0.0f;
            float raw_rpm_r = 0.0f;

            motor_left.get_current_rpm(current_pulse_l, last_pulse_l, delta_time_us, raw_rpm_l);
            motor_right.get_current_rpm(current_pulse_r, last_pulse_r, delta_time_us, raw_rpm_r);

            // JGB37-520 Hardware Quadrature Scaling (x4 division)
            float instant_rpm_l = raw_rpm_l / 4.0f;
            float instant_rpm_r = raw_rpm_r / 4.0f;

            // --- EMA LOW-PASS FILTER FOR QUANTIZATION NOISE REDUCTION ---
            // At 100Hz sampling rate (10ms), a 1-pulse encoder difference can cause an RPM jump of ~4.5 RPM
            static float filtered_rpm_l = 0.0f;
            static float filtered_rpm_r = 0.0f;
            const float ALPHA = 0.15f; // Filter coefficient. Smaller = smoother but higher phase lag (0.1 -> 0.3 is optimal)

            filtered_rpm_l = (ALPHA * instant_rpm_l) + ((1.0f - ALPHA) * filtered_rpm_l);
            filtered_rpm_r = (ALPHA * instant_rpm_r) + ((1.0f - ALPHA) * filtered_rpm_r);

            float rpm_l = filtered_rpm_l;
            float rpm_r = filtered_rpm_r;

            // Extract sensor data safely to make steering decisions
            uint32_t sensor_snapshot[ROBOT_NUM_SENSORS];
            LineSensorCalib calib;
            RobotPhysicalConfig phys_cfg;

            portENTER_CRITICAL(&state->spinlock);
            for (int i = 0; i < ROBOT_NUM_SENSORS; i++) {
                sensor_snapshot[i] = state->adc_raw[i];
            }
            calib = state->line_calib;
            phys_cfg = state->physical_config;
            portEXIT_CRITICAL(&state->spinlock);

            // Execute Line Tracking PID every 5 ticks (50ms / 20Hz)
            if (loop_counter % 5 == 0) {
                // Ensure outer_dt_s reflects the actual period of the outer loop.
                // Normally it is 5 * 10ms = 50ms (0.05s)
                float outer_dt_s = 0.05f; 
                float e2 = line_tracker.compute_e2(sensor_snapshot, calib);
                line_tracker.compute_target_rpm(e2, outer_dt_s, phys_cfg, target_rpm_l, target_rpm_r);
            }

            pid_left.set_target_velocity(target_rpm_l);
            pid_right.set_target_velocity(target_rpm_r);

            float duty_l = pid_left.compute(rpm_l, dt_s);
            float duty_r = pid_right.compute(rpm_r, dt_s);

            motor_left.set_duty_cycle(duty_l);
            motor_right.set_duty_cycle(duty_r);

            // Atomically update state for the Telemetry Task
            portENTER_CRITICAL(&state->spinlock);
            state->encoder_left = current_pulse_l;
            state->encoder_right = current_pulse_r;
            state->pwm_left = duty_l;
            state->pwm_right = duty_r;
            state->target_rpm_left = target_rpm_l;
            state->target_rpm_right = target_rpm_r;
            state->actual_rpm_left = rpm_l;
            state->actual_rpm_right = rpm_r;
            portEXIT_CRITICAL(&state->spinlock);
        }

        last_pulse_l = current_pulse_l;
        last_pulse_r = current_pulse_r;
        last_time_us = current_time_us;
        loop_counter++;

        vTaskDelayUntil(&last_wake_time, freq_ticks);
    }
}
