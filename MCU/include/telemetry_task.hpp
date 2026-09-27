#pragma once

/**
 * @brief Legacy continuous telemetry task routine (kept for reference/backup).
 */
void telemetry_task_routine(void *pvParameters);

/**
 * @brief Power-saving telemetry task routine.
 * 
 * Automatically manages Wi-Fi lifecycle (OFF when running, ON when idle)
 * and transmits a slim ~150-byte status packet (PID, v_ref, sensor weights)
 * at 2Hz when idle.
 */
void telemetry_idle_task_routine(void *pvParameters);
