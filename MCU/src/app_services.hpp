#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace app_services {

// Initialize all hardware peripherals (GPIO, ADC, Motor...)
void hardware_init();

// Start the Wi-Fi service task
void wifi_service_start(UBaseType_t priority);

// Start the Sensor service task
void sensor_service_start(UBaseType_t priority);

} // namespace app_services
