---
trigger: model_decision
---

# ESP32-WROOM-32 Hardware Truth (uPesy DevKit)

## 1. RESERVED & HARDWARE-RESTRICTED GPIOs (ABSOLUTE LIMITS)

> **Never assign outputs or remap peripherals to the following GPIOs.**

| GPIO | Category | Hardware Restriction |
|------|----------|----------------------|
| **6, 7, 8, 9, 10, 11** | Internal SPI Flash | **DO NOT USE.** These pins are permanently connected to the onboard SPI Flash. Accessing them will cause an immediate Guru Meditation (Crash). |
| **34, 35, 36 (VP), 39 (VN)** | Input Only | **INPUT ONLY.** No output driver and no internal pull-up/pull-down resistors. Never use them for PWM, UART TX, SPI output, or any output function. |

---

## 2. STRAPPING PINS (BOOT CONFIGURATION)

> These GPIOs determine the ESP32 boot mode. External circuitry must never force them into an invalid state during power-up or reset.

| GPIO | Strapping Function | Hardware Constraint |
|------|--------------------|---------------------|
| **0** | Boot Mode | Must be **LOW** to enter download mode. Use caution when connecting buttons or external devices. |
| **2** | Boot Mode & Onboard LED | Must remain in a valid boot state during reset. Commonly connected to the onboard blue LED. |
| **5** | SDIO Timing | Must be **HIGH** during boot. Safe for SPI CS or PWM after boot completes. |
| **12** | Flash Voltage Selection | **CRITICAL.** Must be **LOW** during boot. Driving HIGH selects 1.8V Flash mode and prevents normal boot. |
| **15** | Boot Message Configuration | HIGH enables boot log output, LOW suppresses boot messages. |

---

## 3. DEFAULT PERIPHERAL PIN ASSIGNMENT

> Prefer these default IO-MUX pins unless the hardware design requires GPIO Matrix remapping.

| Peripheral | GPIO Assignment | Notes |
|------------|----------------|-------|
| **UART0 (Programming / System Log)** | TX = **1**, RX = **3** | Reserved for USB-UART programming and serial monitor. Avoid using for external peripherals. |
| **UART2** | TX = **17**, RX = **16** | Preferred hardware UART for GPS, LiDAR, and other external modules. |
| **I2C0** | SDA = **21**, SCL = **22** | Default I2C pins. Internal pull-up support available. |
| **VSPI** | MOSI = **23**, MISO = **19**, SCK = **18**, CS = **5** | Preferred SPI bus for maximum compatibility and performance. |

---

## 4. DYNAMIC GPIO MATRIX ALLOCATION

> These assignments depend on the actual hardware design. Always read this table before configuring firmware.

| Function | GPIO | Direction | Notes |
|----------|------|-----------|-------|
| **MOTOR_L_PWM** | `[Assign GPIO]` | Output | Use MCPWM or LEDC |
| **MOTOR_R_PWM** | `[Assign GPIO]` | Output | Use MCPWM or LEDC |
| **ENCODER_L_A** | `[Assign GPIO]` | Input | Use PCNT (Pulse Counter) |
| **ENCODER_L_B** | `[Assign GPIO]` | Input | Use PCNT (Pulse Counter) |
| **ENCODER_R_A** | `[Assign GPIO]` | Input | Use PCNT (Pulse Counter) |
| **ENCODER_R_B** | `[Assign GPIO]` | Input | Use PCNT (Pulse Counter) |

---

## 5. ANALOG INPUTS (FIXED ADC CHANNELS)

> Analog sensors must use dedicated ADC GPIOs. Prefer **ADC1** channels to avoid conflicts with Wi-Fi.

| Analog Sensor | GPIO | ADC Channel | Notes |
|--------------|------|-------------|-------|
| `[Sensor Name]` | `[Assign GPIO]` | `ADC1_CHx` | Prefer ADC1 for Wi-Fi compatibility |

---

## 6. HIGH-SPEED DMA PERIPHERALS (FIXED IO-MUX)

> For high-speed SPI/I2S with DMA, use the native IO-MUX pins below. Do not remap these signals if maximum performance is required.

### VSPI (Up to 80 MHz with DMA)

| Signal | GPIO |
|--------|------|
| SCK | **18** |
| MISO | **19** |
| MOSI | **23** |
| CS | **5** |

### HSPI (Up to 80 MHz with DMA)

| Signal | GPIO |
|--------|------|
| SCK | **14** |
| MISO | **12** |
| MOSI | **13** |
| CS | **15** |