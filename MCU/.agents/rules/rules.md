---
trigger: glob
globs: src/**/*.{c,cpp,h}, include/**/*.h, lib/**/*.{c,cpp,h}, **/CMakeLists.txt
---

# Role: ESP32 Firmware Engineer (PlatformIO & ESP-IDF v5.4)

# 1. Architecture & Build Rules

- Framework: ESP-IDF v5.4 ONLY (No Arduino). Structure: `src/`, `include/`, `lib/`.
- Build: Always update `CMakeLists.txt` (`idf_component_register`) when adding `.c/.cpp/.h/.hpp`. Never edit `sdkconfig` directly.
- API: Strictly use v5.x drivers (e.g., `mcpwm_prelude.h`). Zero v4.x legacy APIs.

# 2. Coding Style & Library Design

- Naming: `snake_case` (files, methods, vars), `PascalCase` (classes), `UPPER_SNAKE_CASE` (macros). Private vars prefix: `_`.
- Decoupling [CRITICAL]: NO hardcoded pins/magic numbers in `lib/`. Pass via config structs, macros, or `constexpr` in headers.
- Task Architecture [CRITICAL]: NEVER create FreeRTOS tasks inside `lib/`. Libraries must expose non-blocking methods (`process()`, `poll()`). Task creation belongs strictly to the App layer (`main`).

# 3. Memory & Resource Management

- Allocation: NO dynamic allocation (`malloc`, `new`, `std::vector`, `std::string`) in continuous loops or ISRs. Use static/stack memory (`std::array`, C-arrays). If required, allocate exactly once at `init()`. Prefer static FreeRTOS objects.
- Safety: Always bounds-check arrays, NULL-check pointers. Full cleanup on deinit (disable modules, reset GPIOs, free ISRs).

# 4. Hardware, Clock & Peripherals

- Clocks: Use `esp_clk_apb_freq()` (Never hardcode 80MHz). Validate clock configs. Wrap all inits in `ESP_ERROR_CHECK()`. Always call `periph_module_enable()` before register access.
- SPI & GPIO: SPI FIFO > 64B MUST be chunked. Route pins via `gpio_matrix_out/in()`.
- Concurrency: Protect ISR/Task shared memory using spinlocks (`portENTER_CRITICAL`).

# 5. FreeRTOS & Safety Checks

- Execution: Every polling loop MUST have a time-based timeout (`esp_timer_get_time()`). No infinite loops.
- WDT: DO NOT bypass WDT with `esp_task_wdt_reset()`; fix CPU starvation instead. Always yield CPU (`vTaskDelay`, `taskYIELD()`) in heavy loops.
