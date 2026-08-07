# AGV Line Following Robot

## Overview

This project is an Autonomous Guided Vehicle (AGV) designed to follow a predefined line path. The repository contains the initial software and hardware development for the AGV platform.

> **Note:** This is the first commit of the project. The architecture, features, documentation, and code structure are expected to change as development progresses.

## Features

- Line following navigation
- Motor speed control
- Sensor data acquisition
- Basic AGV movement logic

## Hardware

Current hardware platform may include:

- Microcontroller (ESP32)
- Line tracking sensors
- DC motors with motor drivers
- Power supply system

## Development Status

- Work in Progress

This repository is currently under active development. Features and documentation are incomplete and may change without notice.

## License

This project is released under the MIT License.

## Line Tracking Logic

The AGV's Line Tracking system is designed using a Cascade Control architecture, combining analog sensor readings, a filtered PD controller, and a Differential Drive kinematic model. Specifically, it consists of the following 4 steps:

### 1. Line Reading & Calibration Logic

- **Data Acquisition:** Data from 5 infrared sensors is continuously read via the ADC converter (using DMA to avoid blocking the CPU).
- **Affine Transformation (Calibration):** Due to varying lighting conditions and unique characteristics of each sensor, the raw ADC values (`adc_raw`) are first clamped to prevent noise. Then, they are linearly mapped to a normalized range (`y_min` to `y_max`) using a slope coefficient `a_coeff`. This coefficient is calculated from the actual measured min/max values of each sensor.

### 2. Lateral Error Calculation ($e_2$)

- Instead of reading the black line in a binary state (white/black), the system utilizes a **Weighted Average** algorithm on the normalized ADC array of the 5 sensors. This approach helps determine the centroid of the line smoothly and accurately.
- The centroid $X$ is calculated as follows:
  $$ X = \frac{2 \cdot (ADC_4 - ADC_0) + (ADC_3 - ADC_1)}{\sum ADC} \cdot 17 $$
- From $X$, the lateral error $e_2$ (in mm) relative to the robot's center is interpolated using an empirical constant:
  $$ e_2 = Coe_1 \cdot X - Coe_2 $$

### 3. PID Controller (Outer Loop)

- The system uses a **PD** (Proportional - Derivative) algorithm to calculate the required angular velocity correction (denoted as $\Delta \omega$).
- A special feature of this PD controller is that the Derivative component (D-Part) integrates a **Low-pass Filter (Derivative Filter)** with a time constant $\tau$. This completely eliminates spikes from the sensors, preventing the robot from stuttering.
- The difference equation calculating $\Delta \omega$ at each cycle $dt$ is:
  $$ \Delta \omega = K*P \cdot e_2 + \frac{2 \cdot K_D \cdot (e_2 - e*{pre}) + (2\tau - dt) \cdot pre_Dpart}{2\tau + dt} $$

### 4. Kinematic Transformation

- After calculating $\Delta \omega$ from the PID algorithm and having the base forward velocity of the vehicle ($V_{ref}$), we apply the kinematic model of a **Differential Drive Robot** to find the necessary angular velocity for each wheel.
- Based on the distance between the 2 wheels ($b$) and the wheel radius ($r$), the velocities of the two wheels ($w_{left}, w_{right}$) in Rad/s are calculated:
  - $w*{left} = \frac{V*{ref} - \frac{b}{2} \cdot \Delta \omega}{r} $
  - $w*{right} = \frac{V*{ref} + \frac{b}{2} \cdot \Delta \omega}{r} $
- Finally, the result (Rad/s) is converted to revolutions per minute (RPM). These two RPM values serve as the **Setpoint** fed into the inner loop controller (Velocity PID) to directly drive the 2 motors to the target speed.
