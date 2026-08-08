#pragma once

#include <stdint.h>
#include <driver/gpio.h>

// ==================== SYSTEM CONFIGURATION ====================

// --- Wi-Fi Target Selection ---
// Set to 1 to connect to Laptop Hotspot, 0 to connect to Home Router
#define USE_LAPTOP_HOTSPOT 1

// --- Wi-Fi Credentials ---
constexpr char WIFI_HOTSPOT_SSID[] = "<YOUR_HOTSPOT_SSID>";
constexpr char WIFI_HOTSPOT_PASS[] = "<YOUR_HOTSPOT_PASSWORD>";

constexpr char WIFI_ROUTER_SSID[] = "<YOUR_ROUTER_SSID>";
constexpr char WIFI_ROUTER_PASS[] = "<YOUR_ROUTER_PASSWORD>";

// --- Telemetry Networking ---
#if USE_LAPTOP_HOTSPOT
// IP of the laptop when it is hosting a mobile hotspot (usually 192.168.137.1 on Windows)
constexpr char UDP_TARGET_IP[] = "<YOUR_HOTSPOT_IP>";
#else
constexpr char UDP_TARGET_IP[] = "<YOUR_ROUTER_IP>";
#endif

constexpr uint16_t UDP_TARGET_PORT = 54321;
constexpr uint16_t UDP_LISTEN_PORT = 54322;


// ==================== HARDWARE PIN MAPPING ====================

// --- TB6612FNG Motor Driver Pins ---
constexpr int PIN_MOTOR_L_PWM = 25;
constexpr int PIN_MOTOR_L_IN1 = 26;
constexpr int PIN_MOTOR_L_IN2 = 27;

constexpr int PIN_MOTOR_R_PWM = 14;
constexpr int PIN_MOTOR_R_IN1 = 16;
constexpr int PIN_MOTOR_R_IN2 = 17;

// --- JGB37-520 Encoder Pins (Must support interrupts) ---
constexpr int PIN_ENC_L_A = 18;
constexpr int PIN_ENC_L_B = 19;

constexpr int PIN_ENC_R_A = 22;
constexpr int PIN_ENC_R_B = 21;

// --- 5-Channel Line Sensor Pins ---
// Mapped internally via ADC Channels in adc_dma.hpp:
// SENSOR 1: GPIO32 (ADC1_CH4)
// SENSOR 2: GPIO33 (ADC1_CH5)
// SENSOR 3: GPIO34 (ADC1_CH6)
// SENSOR 4: GPIO35 (ADC1_CH7)
// SENSOR 5: GPIO36 (ADC1_CH0)

// --- HX711 Loadcell Pins ---
constexpr gpio_num_t PIN_LOADCELL_DT = GPIO_NUM_23;
constexpr gpio_num_t PIN_LOADCELL_SCK = GPIO_NUM_13;

// --- Wi-Fi Switch Pin ---
constexpr gpio_num_t PIN_WIFI_SWITCH = GPIO_NUM_4;


// ==================== INITIAL DEFAULT PID & PHYSICS ====================
// These values are only used when NVS is empty (first boot or wiped)

constexpr float DEFAULT_PHYS_WHEEL_BASE_MM = 150.0f;
constexpr float DEFAULT_PHYS_WHEEL_RADIUS_MM = 30.0f;

// Left Motor Velocity PID Defaults
constexpr float DEFAULT_KP_L = 1.0f;
constexpr float DEFAULT_KI_L = 0.1f;
constexpr float DEFAULT_KD_L = 0.05f;

// Right Motor Velocity PID Defaults
constexpr float DEFAULT_KP_R = 1.0f;
constexpr float DEFAULT_KI_R = 0.1f;
constexpr float DEFAULT_KD_R = 0.05f;

// Line Tracker PD Defaults
constexpr float DEFAULT_KP = 0.0f;
constexpr float DEFAULT_KD = 0.0f;
constexpr float DEFAULT_PID_TAU = 0.05f;
