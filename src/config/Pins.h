#pragma once

#include <Arduino.h>
#include <Wire.h>

// TODO: Replace all placeholder pin assignments with actual ESP32-S3 board wiring.

// Define STEP1 as Azimuth, STEP2 as Elevation

// Azimuth axis pins
constexpr uint8_t AZ_STEP_PIN = 1;
constexpr uint8_t AZ_DIR_PIN = 2;
// constexpr uint8_t AZ_EN_PIN = -1; //Tied High
// constexpr uint8_t AZ_DIAG_PIN = 7; //Not Connected

// Elevation axis pins
constexpr uint8_t EL_STEP_PIN = 8;
constexpr uint8_t EL_DIR_PIN = 9;
// constexpr uint8_t EL_EN_PIN = 10;
// constexpr uint8_t EL_DIAG_PIN = 11;

// TMC2209 UART configuration
inline HardwareSerial& AZ_TMC_SERIAL = Serial1;
constexpr uint8_t AZ_TMC_UART_TX_PIN = 14;
constexpr uint8_t AZ_TMC_UART_RX_PIN = 15;

inline HardwareSerial& EL_TMC_SERIAL = Serial2;
constexpr uint8_t EL_TMC_UART_TX_PIN = 18;
constexpr uint8_t EL_TMC_UART_RX_PIN = 17;
// TODO: If using RS485 transceivers or UART muxing, add DE/RE control here.

// AS5600 I2C configuration
inline TwoWire& AZ_AS5600_I2C = Wire;
constexpr uint8_t AZ_AS5600_SDA_PIN = 8;
constexpr uint8_t AZ_AS5600_SCL_PIN = 9;

inline TwoWire& EL_AS5600_I2C = Wire;
constexpr uint8_t AZ_AS5600_SDA_PIN = 10;
constexpr uint8_t AZ_AS5600_SCL_PIN = 11;


// NOTE: AS5600 has a fixed address (0x36).
// TODO: Use I2C mux or separate bus if both sensors are truly AS5600 and share the same bus.
constexpr uint8_t AS5600_ADDR_AZ = 0x36;
constexpr uint8_t AS5600_ADDR_EL = 0x36;

// ESTOP Pin
constexpr uint8_t ESTOP_PIN = 32;
constexpr uint8_t BOOT_PIN = 0;

// uBLOX M10 + Compass
constexpr uint8_t GPS_UART_TX_PIN = 36;
constexpr uint8_t GPS_UART_RX_PIN = 37;
constexpr uint8_t QMC_5883L_I2C = 0x0D;

// WS2812-B
constexpr uint8_t LED_PIN = 3;
