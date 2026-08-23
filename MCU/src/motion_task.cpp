/**
 * @file motion_task.cpp
 * @brief FreeRTOS task for motion control, sensor reading, and PID execution.
 *
 * This module integrates the encoder readings, line sensor data, and executes
 * the velocity and steering PID controllers to drive the motors.
 */
#include "../include/motion_task.hpp"
#include "../include/autonomous_control.hpp"
#include "../include/main.hpp"
#include "../include/manual_control.hpp"
#include "../include/motion_strategy.hpp"
#include "../include/shared_state.hpp"
#include "../lib/tb6612_encoder/tb6612_encoder.hpp"
#include "../lib/velocity_pid/velocity_pid.hpp"
#include <driver/gpio.h>
#include <esp_timer.h>

#if CONTROL_MODE == 1
static ManualControl motion_controller;
#else
static AutonomousControl motion_controller;
#endif

extern Tb6612Encoder motor_left;
extern Tb6612Encoder motor_right;
extern VelocityPid pid_left;
extern VelocityPid pid_right;

/**
 * @brief Main motion control task routine.
 *
 * Runs at a strict 100Hz frequency. Reads encoders, calculates instant RPM,
 * applies EMA filtering for quantization noise reduction, runs Line Tracking
 * PID at 20Hz, and updates Motor Velocity PIDs.
 *
 * @param pvParameters Pointer to the global SharedRobotState.
 */
void motion_task_routine(void *pvParameters) {
  SharedRobotState *state = static_cast<SharedRobotState *>(pvParameters);
  
  float target_rpm_l = 0.0f;
  float target_rpm_r = 0.0f;
  float current_e2 = 0.0f;

  bool prev_system_running = false;

  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t freq_ticks = pdMS_TO_TICKS(10); // 100Hz

  int64_t last_time_us = esp_timer_get_time();
  int64_t last_pulse_l = 0;
  int64_t last_pulse_r = 0;
  int64_t enc_offset_l = 0;
  int64_t enc_offset_r = 0;

  motor_left.get_pulse_count(last_pulse_l);
  motor_right.get_pulse_count(last_pulse_r);

  uint32_t loop_counter = 0;
  float target_rpm_l = 0.0f;
  float target_rpm_r = 0.0f;

  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = (1ULL << GPIO_NUM_0);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&io_conf);

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

      motor_left.get_current_rpm(current_pulse_l, last_pulse_l, delta_time_us,
                                 raw_rpm_l);
      motor_right.get_current_rpm(current_pulse_r, last_pulse_r, delta_time_us,
                                  raw_rpm_r);

      // JGB37-520 Hardware Quadrature Scaling (x4 division)
      // Optimized: Multiplication by 0.25f is faster than division by 4.0f
      float instant_rpm_l = raw_rpm_l * 0.25f;
      float instant_rpm_r = raw_rpm_r * 0.25f;

      /**
       * @note EMA LOW-PASS FILTER FOR QUANTIZATION NOISE REDUCTION
       * At 100Hz sampling rate (10ms dt), a 1-pulse encoder difference can
       * cause an RPM jump of ~4.5 RPM due to quantization. The EMA filter
       * smooths this out. Alpha is the filter coefficient (smaller = smoother
       * but higher phase lag). 0.15 is chosen as an optimal balance for this
       * encoder's PPR.
       */
      static float filtered_rpm_l = 0.0f;
      static float filtered_rpm_r = 0.0f;
      constexpr float ALPHA =
          1.0f; // EMA filter coefficient for quantization noise

      filtered_rpm_l =
          (ALPHA * instant_rpm_l) + ((1.0f - ALPHA) * filtered_rpm_l);
      filtered_rpm_r =
          (ALPHA * instant_rpm_r) + ((1.0f - ALPHA) * filtered_rpm_r);

      float rpm_l = filtered_rpm_l;
      float rpm_r = filtered_rpm_r;

      StateSnapshot snap;
      portENTER_CRITICAL(&state->spinlock);
      for (int i = 0; i < ROBOT_NUM_SENSORS; i++) {
        snap.adc_raw[i] = state->adc_raw[i];
      }
      snap.line_calib = state->line_calib;
      snap.physical_config = state->physical_config;
      snap.manual_cmd_l = state->manual_cmd_l;
      snap.manual_cmd_r = state->manual_cmd_r;
      snap.loadcell_weight = state->loadcell_weight;
      snap.track_config = state->track_config;
      portEXIT_CRITICAL(&state->spinlock);

      // Pass the *raw* un-decimated pulses so we get accurate displacement
      int64_t relative_pulse_l = current_pulse_l - enc_offset_l;
      int64_t relative_pulse_r = current_pulse_r - enc_offset_r;
      snap.encoder_l = relative_pulse_l;
      snap.encoder_r = relative_pulse_r;

      // Compute strategy (Manual or Autonomous)
      MotionOutput m_out = motion_controller.compute(snap, dt_s, loop_counter);
      target_rpm_l = m_out.target_rpm_left;
      target_rpm_r = m_out.target_rpm_right;
      current_e2 = m_out.current_e2;

      // #if CONTROL_MODE == 1
      //       // DEBUG MODE: Bypass PID, treat manual target as raw PWM duty
      //       cycle (-100 to 100) float duty_l = target_rpm_l; float duty_r =
      //       target_rpm_r;
      // #else

      bool current_system_running = false;
      bool current_soft_stop_req = false;
      portENTER_CRITICAL(&state->spinlock);
      current_system_running = state->system_running;
      current_soft_stop_req = state->soft_stop_request;
      portEXIT_CRITICAL(&state->spinlock);

      // 1. BOOT Button Polling
      static uint32_t btn_press_ticks = 0;
      if (gpio_get_level(GPIO_NUM_0) == 0) {
        btn_press_ticks++;
      } else {
        // If released after a short press (debounce ~5 ticks = 50ms)
        if (btn_press_ticks > 5) {
          portENTER_CRITICAL(&state->spinlock);
          state->system_running = true; // Button ONLY starts the car
          state->soft_stop_request = false;
          current_system_running = true;
          current_soft_stop_req = false;
          portEXIT_CRITICAL(&state->spinlock);
        }
        btn_press_ticks = 0;
      }

      // 2. Handle System Start (Rising Edge)
      if (current_system_running && !prev_system_running) {
        enc_offset_l = current_pulse_l;
        enc_offset_r = current_pulse_r;
        motion_controller.reset();
        ESP_LOGI(TAG, "System Started: Encoders and Controller reset.");
      }
      prev_system_running = current_system_running;

      float duty_l = 0.0f;
      float duty_r = 0.0f;

      if (!current_system_running) {
        // IDLE MODE: Coasting
        pid_left.reset();
        pid_right.reset();
        target_rpm_l = 0.0f;
        target_rpm_r = 0.0f;
        duty_l = 0.0f;
        duty_r = 0.0f;
      } else {
        // RUNNING MODE
        MotionOutput m_out;

        if (current_soft_stop_req) {
          // Soft stop requested by UDP
          m_out.target_rpm_left = 0.0f;
          m_out.target_rpm_right = 0.0f;
        } else {
          m_out = motion_controller.compute(snap, dt_s, loop_counter);
          current_e2 = m_out.current_e2;
        }

        target_rpm_l = m_out.target_rpm_left;
        target_rpm_r = m_out.target_rpm_right;

        // Check if we have successfully soft-stopped (ONLY if soft stop was
        // requested)
        if (current_soft_stop_req && target_rpm_l == 0.0f &&
            target_rpm_r == 0.0f) {
          if (std::abs(rpm_l) < 5.0f && std::abs(rpm_r) < 5.0f) {
            // Fully stopped, go to IDLE
            portENTER_CRITICAL(&state->spinlock);
            state->system_running = false;
            state->soft_stop_request = false;
            current_system_running = false;
            portEXIT_CRITICAL(&state->spinlock);
          }
        }
        /// \note Keep PID tracking active even when target = 0 so PID can generate positive/negative duty to brake the vehicle,
        /// utilizing the Slew Rate Limiter and Deadband features of VelocityPID.
        pid_left.set_target_velocity(target_rpm_l);
        pid_right.set_target_velocity(target_rpm_r);
        duty_l = pid_left.compute(rpm_l, dt_s);
        duty_r = pid_right.compute(rpm_r, dt_s);
      }
      // #endif

      motor_left.set_duty_cycle(duty_l);
      motor_right.set_duty_cycle(duty_r);

      // Atomically update state for the Telemetry Task
      portENTER_CRITICAL(&state->spinlock);
      state->encoder_left = relative_pulse_l;
      state->encoder_right = relative_pulse_r;
      state->pwm_left = duty_l;
      state->pwm_right = duty_r;
      state->target_rpm_left = target_rpm_l;
      state->target_rpm_right = target_rpm_r;
      state->actual_rpm_left = rpm_l;
      state->actual_rpm_right = rpm_r;
      state->current_e2 = current_e2;
      portEXIT_CRITICAL(&state->spinlock);
    }

    last_pulse_l = current_pulse_l;
    last_pulse_r = current_pulse_r;
    last_time_us = current_time_us;
    loop_counter++;

    vTaskDelayUntil(&last_wake_time, freq_ticks);
  }
}
