#pragma once

// --- Pin Definitions ---
#define DAC_PIN 25

// --- INA219 Sensor Configuration ---
#define INA219_ADDRESS 0x40
#define SHUNT_RESISTOR_OHMS 0.1 // 100 mOhm shunt resistor
#define MAXIMUM_BUS_VOLTAGE_INA219 25.0 // Safety limit for INA219

// --- Default PID Tuning Parameters ---
#define DEFAULT_KP 20.0
#define DEFAULT_KI 5.0
#define DEFAULT_KD 1.0

// --- Buck Converter Parameters ---
#define BUCK_FEEDBACK_VOLTAGE 1.25 // Feedback voltage for the buck converter
#define DAC_SAFETY_VALUE (int)((BUCK_FEEDBACK_VOLTAGE / 3.3) * 255.0) // Maximum value for DAC output