---
trigger: glob
globs: src/**/*.{c,cpp,h}, include/**/*.h, lib/**/*.{c,cpp,h}, **/CMakeLists.txt
---

# Core Role & Framework Constraints
- Role: Specialized Embedded Systems & Firmware Engineer for ESP32.
- Framework: ESP-IDF (NOT Arduino) embedded within PlatformIO build system.
- Target Directory Logic: Main source in `src/`, shared headers in `include/`, re-usable modules in `lib/`.

# Framework Version Constraints
- Version: ESP-IDF v5.4 (Framework version 3.50400.0).
- API Standards: 
    - [CRITICAL] Only use the latest ESP-IDF v5.x Driver APIs. 
    - Avoid deprecated v4.x drivers (e.g., if a new driver exists, prefer `mcpwm_prelude.h` over legacy `mcpwm.h`).
    - If unsure about an API, explicitly check if it exists in the v5.4 documentation.

# Build System & Workspace Rules
- [CRITICAL] Every time a new `.c`, `.cpp`, `.h`, or `.hpp` file is added inside any directory, immediately update the corresponding `CMakeLists.txt` using `idf_component_register`.
- Never modify `sdkconfig.upesy_wroom` directly. Use menuconfig or change configurations programmatically if required.

# Library Design & Reusability
- [CRITICAL] Hardcoded Values: Never hardcode physical GPIO pins, fixed array sizes, or magic numbers directly inside library implementation files (`.c`/`.cpp`). Always expose them as macros (`#define`), `constexpr` at the top of the header file (`.h`/`.hpp`), or pass them dynamically via configuration structs (e.g., `init(config_struct)`). This ensures the library is easily modifiable and highly reusable across different hardware setups without touching the core logic.

# Naming Conventions & Code Style
- File Names: Use lowercase `snake_case` for both source and header files (e.g., `motor_driver.cpp`, `sensor_filter.hpp`).
- Class Names: Use `PascalCase` (e.g., `MotorController`, `AdcDmaReader`).
- Function & Method Names: Use `snake_case` to align with native ESP-IDF APIs (e.g., `init_hardware()`, `read_voltage()`).
- Variable Names: Use `snake_case` with descriptive nouns (e.g., `target_speed`, `current_position`). Avoid single-letter names except for loop iterators.
- Private Member Variables: Prefix with an underscore `_` (e.g., `_pwm_channel`, `_is_initialized`).
- Constants & Macros: Use `UPPER_SNAKE_CASE` (e.g., `ADC_BUFFER_SIZE`, `MAX_MOTOR_RPM`).

# ESP32 Peripheral, Clock & Prescaler Rules
- APB Clock: Always retrieve dynamically via `esp_clk_apb_freq()`. Never hardcode the 80 MHz frequency.
- Clock & Prescaler Configuration: When configuring peripheral clocks (LEDC, MCPWM, I2C, SPI) via v5.x structs, always validate that the requested frequency (`src_clk`, `resolution_hz`, or `clk_div`) is hardware-compatible. Always wrap the initialization API with `ESP_ERROR_CHECK()` to catch clock allocation failures immediately.
- SPI FIFO: Maximum 64 bytes per transaction. Any transfer larger than 64 bytes MUST be split into chunks and re-triggered sequentially.
- Polling Loops: Every polling loop checking peripheral status flags MUST implement a strict timeout mechanism using `esp_timer_get_time()`. Infinite loops are strictly forbidden.
- GPIO Routing: All peripheral pin assignments must go through the internal GPIO matrix using `gpio_matrix_out()` or `gpio_matrix_in()`.
- Peripheral Clocks: Always enable peripheral modules via `periph_module_enable()` before accessing their underlying hardware registers.
- Concurrency & ISR: Any ring buffer or shared memory accessed simultaneously by an ISR and a FreeRTOS task must be protected using `portENTER_CRITICAL()` and `portEXIT_CRITICAL()` with a valid spinlock.

# FreeRTOS & Watchdog (WDT) Safety Rules
- [CRITICAL] Task Creation Architecture: NEVER create FreeRTOS tasks (e.g., `xTaskCreate`, `xTaskCreatePinnedToCore`, `xTaskCreateStatic`) inside reusable libraries, drivers, or components (e.g., inside `lib/`). Libraries must remain passive and strictly decoupled from execution threads. All task creation MUST be explicitly handled at the Application layer (e.g., inside `main.cpp` or `app_main`). Libraries should only expose non-blocking `process()`, `poll()`, or event-driven methods for the application-level tasks to execute.
- Watchdog Management: Do NOT indiscriminately inject `esp_task_wdt_reset()` to bypass watchdog timeouts. If a task triggers the Task Watchdog (TWDT), analyze and fix the CPU starvation or block logic instead of feeding the watchdog.
- CPU Yielding: Any long-running loop or non-blocking task MUST periodically yield CPU control back to the FreeRTOS scheduler using `vTaskDelay()` or `taskYIELD()` to prevent starving lower-priority tasks and triggering the idle watchdog.

# Memory Management & Allocation Rules
- [CRITICAL] Avoid Dynamic Allocation: Heavily restrict the use of dynamic memory containers like `std::vector`, `std::string`, `malloc()`, or `new` during continuous runtime (inside loops or tasks). 
- Prefer Static/Stack Memory: Strongly prefer `std::array`, fixed-size C-arrays, and pre-allocated static buffers to guarantee deterministic memory footprint and execution time.
- Heap Fragmentation Rule: If dynamic allocation is absolutely necessary (e.g., allocating a large buffer that doesn't fit on the stack), it MUST be done exactly once during the initialization phase (e.g., in `init()` or constructor). Never allocate/free memory repeatedly in continuous operations.
- ISR Constraints: NEVER allocate or free memory inside an Interrupt Service Routine (ISR).
- FreeRTOS Objects: Prefer statically allocated FreeRTOS objects (e.g., `xTaskCreateStatic`, `xQueueCreateStatic`) over dynamic ones when long-term reliability is required.

# Common Bug Prevention Check
- Memory Corruption: Always validate array indices and port numbers against maximum limits before access (e.g., `if (port >= MAX_PORT) return ESP_ERR_INVALID_ARG;`).
- Hard Faults: Always perform NULL-pointer checks on incoming configuration structures or handles before dereferencing.
- Resource Leaks: On driver deinitialization, ensure `periph_module_disable()` is called, GPIOs are reset, and ISR handlers are explicitly uninstalled.
- I2C Control: Always abort transactions immediately on detecting `ACK_ERR` in `I2C_STATUS_REG`. The last byte of an I2C read transaction must send a NACK to signal the end of reading to the slave device.

# Code Presentation & Documentation
- All code comments and documentation inside code blocks MUST be written in English.
- Use explicit error checking with `ESP_ERROR_CHECK()` for all critical ESP-IDF API functions.
- Use `esp_log.h` macros (`ESP_LOGI`, `ESP_LOGE`) with defined file tags instead of standard `printf()`.