/**
 * @file line_tracker.hpp
 * @brief Line Tracking PID Controller with derivative filtering.
 */
#pragma once

#include <stdint.h>
#include "shared_state.hpp"

/**
 * @brief LineTracker class for computing lateral error and target wheel RPMs.
 * 
 * Uses a PD control law combined with a discrete derivative filter (tau) 
 * to smoothly track a path based on a 5-channel analog sensor array.
 */
class LineTracker {
public:
    LineTracker() = default;

    /**
     * @brief Computes the lateral error e2 based on ADC readings and calibration.
     * 
     * @param adc_raw Array of 5 ADC readings.
     * @param calib Calibration constants for the line sensors.
     * @return float Calculated lateral error e2 in mm.
     */
    float compute_e2(const uint32_t adc_raw[ROBOT_NUM_SENSORS], const LineSensorCalib &calib);

    /**
     * @brief Computes the target RPM for both wheels based on tracking error.
     * 
     * @param e2 The current lateral error in mm.
     * @param dt_s The time elapsed since the last computation in seconds.
     * @param cfg Physical configuration and PID gains.
     * @param out_rpm_l Reference to store the target left RPM.
     * @param out_rpm_r Reference to store the target right RPM.
     */
    void compute_target_rpm(float e2, float dt_s, const RobotPhysicalConfig &cfg, float &out_rpm_l, float &out_rpm_r);

    /**
     * @brief Resets the internal state (D-part, previous error) of the controller.
     */
    void reset();

private:
    float _prev_e2{0.0f};      /**< Previous lateral error in mm */
    float _prev_d_part{0.0f};  /**< Previous derivative filter state */
};
