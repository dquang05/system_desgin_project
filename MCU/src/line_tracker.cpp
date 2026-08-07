#include "../include/line_tracker.hpp"
#include <cmath>
#include <algorithm>

float LineTracker::compute_e2(const uint32_t adc_raw[ROBOT_NUM_SENSORS], const LineSensorCalib &calib) {
    float adc_calib[ROBOT_NUM_SENSORS];
    float sum_adc = 0.0f;

    // Affine Transformation (Calibration)
    for (int i = 0; i < ROBOT_NUM_SENSORS; i++) {
        float a_coeff = static_cast<float>(calib.y_max - calib.y_min) / 
                        static_cast<float>(calib.x_max[i] - calib.x_min[i]);
        
        // Clamp below to x_min to prevent negative values if raw < x_min
        float raw_clamped = std::max(static_cast<float>(adc_raw[i]), static_cast<float>(calib.x_min[i]));
        
        adc_calib[i] = calib.y_min + a_coeff * (raw_clamped - calib.x_min[i]);
        sum_adc += adc_calib[i];
    }

    if (sum_adc <= 0.001f) {
        return _pre_e2; // Avoid division by zero, return previous error
    }

    // Weighted Average
    float X = (2.0f * (adc_calib[4] - adc_calib[0]) + (adc_calib[3] - adc_calib[1])) * 17.0f / sum_adc;
    
    float e2 = calib.line_coe_1 * X - calib.line_coe_2;
    return e2;
}

void LineTracker::compute_target_rpm(float e2, float dt_s, const RobotPhysicalConfig &cfg, float &out_rpm_l, float &out_rpm_r) {
    // PD Control with Derivative Filter
    float delta_w = cfg.kp * e2 + 
                    2.0f * cfg.kd * (e2 - _pre_e2) / (2.0f * cfg.pid_tau + dt_s) + 
                    (2.0f * cfg.pid_tau - dt_s) * _pre_Dpart / (2.0f * cfg.pid_tau + dt_s);

    _pre_Dpart = 2.0f * cfg.kd * (e2 - _pre_e2) / (2.0f * cfg.pid_tau + dt_s) + 
                 (2.0f * cfg.pid_tau - dt_s) * _pre_Dpart / (2.0f * cfg.pid_tau + dt_s);

    _pre_e2 = e2;

    // Optional: Limit delta_w if necessary (as seen in some parts of the reference code)
    if (std::abs(delta_w) >= 3.0f) {
        delta_w = 3.0f * (delta_w >= 0.0f ? 1.0f : -1.0f);
    }

    // Calculate wheel velocities in m/s or rad/s?
    // In reference code: w_left = (V_REF - (L_WHEEL / 2.0f) * delta_w) / (D_WHEEL / 2.0f);
    // So target wheel angular velocity in rad/s:
    float w_left_rads = (cfg.v_ref - (cfg.wheel_base_mm / 2.0f) * delta_w) / cfg.wheel_radius_mm;
    float w_right_rads = (cfg.v_ref + (cfg.wheel_base_mm / 2.0f) * delta_w) / cfg.wheel_radius_mm;

    // Convert rad/s to RPM
    // RPM = rad/s * (60 / (2 * pi))
    const float RADS_TO_RPM = 60.0f / (2.0f * (float)M_PI);
    out_rpm_l = w_left_rads * RADS_TO_RPM;
    out_rpm_r = w_right_rads * RADS_TO_RPM;
}

void LineTracker::reset() {
    _pre_e2 = 0.0f;
    _pre_Dpart = 0.0f;
}
