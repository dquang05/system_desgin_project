# 1. System Core Configuration

- Debug: Serial Wire mode (using ST-Link).
- RCC: Use an external crystal oscillator (Crystal/Ceramic Resonator) for stable operation at 72 MHz.

# 2. ADC Configuration (5 Line Sensor Channels)

- Mode:
  - Independent Mode
  - Scan Conversion Mode = Enabled
  - Continuous Conversion Mode = Enabled
- Number of Channels: 5 channels (Rank 1–5 assigned to the corresponding ADC channels).
- Sampling Time: 71.5 Cycles (to improve signal stability and reduce noise).
- Data Alignment: Right Alignment (ADC result range: 0–4095).
- DMA:
  - ADC1 → DMA1 Channel 1
  - Direction = Peripheral to Memory
  - Mode = Circular
  - Data Width = Half Word

# 3. Motor & Encoder Configuration (Timers & GPIO)

## PWM (Motor Speed Control)

- Timer: TIM4 (Channel 1 & Channel 2)
- PWM Frequency: 20 kHz
- ARR = 3599
- Expected Output Pins:
  - PB6 → TIM4_CH1
  - PB7 → TIM4_CH2

## Encoder (Speed Feedback)

### Left Encoder

- TIM2 (Encoder Mode, TI1 & TI2)
- Pins:
  - PA0 → TIM2_CH1
  - PA1 → TIM2_CH2

### Right Encoder

- TIM3 (Encoder Mode, TI1 & TI2)
- Pins:
  - PA6 → TIM3_CH1
  - PA7 → TIM3_CH2

## Motor Direction Control

- 4 GPIO pins configured as:
  - Output Mode
  - High Speed

Recommended pins:

- PB12
- PB13
- PB14
- PB15

This selection keeps the UART interface (PA9, PA10) available for PID debugging and runtime monitoring.

# 4. Pinout Summary (Proposed)

| Function | Pin(s) | Notes |
|----------|---------|--------|
| Line Sensor 1–5 | PA0–PA4 (or other ADC-capable pins) | Analog Mode |
| Left / Right PWM | PB6 / PB7 | TIM4_CH1, TIM4_CH2 |
| Left Encoder (A, B) | PA0 / PA1 | TIM2_CH1, TIM2_CH2 |
| Right Encoder (A, B) | PA6 / PA7 | TIM3_CH1, TIM3_CH2 |
| Motor Direction Control | PB12, PB13, PB14, PB15 | GPIO Output |
| Debug / Programming | PA13, PA14 | SWDIO, SWCLK |