---
trigger: always_on
glob:
description:
---

# ESP32 ADC DMA & FreeRTOS Lessons Learned

- **ADC Calibration**: ESP32 (classic) only supports `Line Fitting` calibration (`adc_cali_create_scheme_line_fitting`). Using `Curve Fitting` will fail as it is disabled in `adc_cali_scheme.h` for this chip.
- **ESP-IDF v5 Spinlock Initialization**: Avoid using internal macros like `vPortCPUInitializeMutex` in modern ESP-IDF v5.x to initialize a Spinlock (`portMUX_TYPE`). Instead, initialize it directly using the macro `portMUX_INITIALIZER_UNLOCKED` (e.g., `portMUX_TYPE _spinlock = portMUX_INITIALIZER_UNLOCKED;`).
- **ADC DMA Zero Queue Architecture**: To minimize overhead and avoid FreeRTOS RingBuffer or xQueue complexities, parse the ADC continuous DMA buffer locally on the stack and safely copy the latest complete frame to a shared struct protected by a Spinlock (`portENTER_CRITICAL` / `portEXIT_CRITICAL`). This prevents race conditions with minimal latency.
- **ADC Continuous Read Timeout**: The `timeout_ms` parameter in `adc_continuous_read()` expects raw milliseconds, NOT FreeRTOS ticks. Passing `pdMS_TO_TICKS(10)` evaluates to ~1ms (or 0), causing an immediate timeout and potential Task Watchdog (TWDT) crash due to a tight spinning loop.
- **Task Watchdog & Yielding**: When an infinite loop task polls for data (e.g., getting `ESP_ERR_TIMEOUT`), it MUST explicitly yield the CPU (e.g., using `vTaskDelay(pdMS_TO_TICKS(10))`) to prevent TWDT CPU starvation.
- **PlatformIO ESP-IDF Component Registration**: Any custom library placed in the `lib/` directory MUST have a `CMakeLists.txt` file calling `idf_component_register()` to be compiled and have its include paths exported properly to the rest of the project.
