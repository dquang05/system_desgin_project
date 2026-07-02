#include "../include/motion_task.hpp"
#include "../include/shared_state.hpp"
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

            // --- BỘ LỌC LOW-PASS FILTER (EMA) KHỬ NHIỄU LƯỢNG TỬ HÓA ---
            // Ở tần số lấy mẫu 100Hz (10ms), chênh lệch 1 xung encoder có thể làm RPM giật ~4.5 vòng/phút
            static float filtered_rpm_l = 0.0f;
            static float filtered_rpm_r = 0.0f;
            const float ALPHA = 0.15f; // Hệ số lọc. Càng nhỏ càng mượt nhưng trễ pha hơn (0.1 -> 0.3 là đẹp)

            filtered_rpm_l = (ALPHA * instant_rpm_l) + ((1.0f - ALPHA) * filtered_rpm_l);
            filtered_rpm_r = (ALPHA * instant_rpm_r) + ((1.0f - ALPHA) * filtered_rpm_r);

            float rpm_l = filtered_rpm_l;
            float rpm_r = filtered_rpm_r;

            // Extract sensor data safely to make steering decisions
            uint32_t sensor_snapshot[ROBOT_NUM_SENSORS];
            portENTER_CRITICAL(&state->spinlock);
            for (int i = 0; i < ROBOT_NUM_SENSORS; i++) {
                sensor_snapshot[i] = state->adc_raw[i];
            }
            portEXIT_CRITICAL(&state->spinlock);
            (void)sensor_snapshot; // Silence unused variable warning

            // TODO: Execute AMR Line Following Logic to define target_rpm_l & target_rpm_r
            // TODO: Khai báo tốc độ tối đa theo thông số thực tế của motor JGB37-520 (VD: 330, 600)
            const float MAX_RPM = 330.0f; 

            // TEST CLOSED-LOOP: Motor A dừng, Motor B chạy 50% vận tốc thật sự (RPM)
            float target_rpm_l = 0.0f; 
            float target_rpm_r = MAX_RPM * 0.5f; // 50% Tốc độ thực tế

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

        vTaskDelayUntil(&last_wake_time, freq_ticks);
    }
}
