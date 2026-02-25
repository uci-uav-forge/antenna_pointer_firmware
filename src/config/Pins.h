#pragma once

#include <Arduino.h>
#include <Wire.h>

// TODO: Replace all placeholder pin assignments with actual ESP32-S3 board wiring.

// Azimuth axis pins
constexpr uint8_t AZ_STEP_PIN = 4;
constexpr uint8_t AZ_DIR_PIN = 5;
constexpr uint8_t AZ_EN_PIN = 6;
constexpr uint8_t AZ_DIAG_PIN = 7;

// Elevation axis pins
constexpr uint8_t EL_STEP_PIN = 8;
constexpr uint8_t EL_DIR_PIN = 9;
constexpr uint8_t EL_EN_PIN = 10;
constexpr uint8_t EL_DIAG_PIN = 11;

// TMC2209 UART configuration
inline HardwareSerial& TMC_SERIAL = Serial1;
constexpr uint8_t TMC_UART_TX_PIN = 17;
constexpr uint8_t TMC_UART_RX_PIN = 18;
// TODO: If using RS485 transceivers or UART muxing, add DE/RE control here.

// AS5600 I2C configuration
inline TwoWire& AS5600_I2C = Wire;
constexpr uint8_t AS5600_SDA_PIN = 1;
constexpr uint8_t AS5600_SCL_PIN = 2;

// NOTE: AS5600 has a fixed address (0x36).
// TODO: Use I2C mux or separate bus if both sensors are truly AS5600 and share the same bus.
constexpr uint8_t AS5600_ADDR_AZ = 0x36;
constexpr uint8_t AS5600_ADDR_EL = 0x36;
