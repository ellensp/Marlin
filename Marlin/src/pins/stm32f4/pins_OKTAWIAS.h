/**
 * Marlin 3D Printer Firmware
 * Adapted for custom board: Oktawia Blackpill (STM32F411CEU6)
 */
#pragma once

#include "env_validate.h"

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "oktawia_blackpill"
#endif

#ifndef DEFAULT_MACHINE_NAME
  #define DEFAULT_MACHINE_NAME "Oktawias Printer"
#endif

#define DISABLE_DEBUG                      false
#define ALLOW_STM32F4

// ---------------------------
// EEPROM Configuration
// ---------------------------
#if NO_EEPROM_SELECTED
  #define IIC_BL24CXX_EEPROM
#endif

// ---------------------------
// LED Indicator
// ---------------------------
#define LED_PIN                             PC13

// ---------------------------
// Limit Switches
// ---------------------------
#define X_MIN_PIN                           PB10
#define Y_MIN_PIN                           PB2
#define Z_MIN_PIN                           PB1   // Z-axis min endstop
#define Z_MIN2_PIN                          PB0   // Second Z-axis min endstop

// ---------------------------
// Temperature Sensors
// ---------------------------
#define TEMP_0_PIN                          PA4   // Hotend thermistor
#define TEMP_BED_PIN                        PA0   // Heated bed thermistor

// ---------------------------
// Stepper Drivers (TMC2208 UART)
// ---------------------------
// X Axis
#define X_STEP_PIN                          PB13
#define X_DIR_PIN                           PB14
#define X_ENABLE_PIN                        -1
#define X_SERIAL_TX_PIN                     PB12
#define X_SERIAL_RX_PIN          X_SERIAL_TX_PIN

// Y Axis
#define Y_STEP_PIN                          PA8
#define Y_DIR_PIN                           PA9
#define Y_ENABLE_PIN                        -1
#define Y_SERIAL_TX_PIN                     PB15
#define Y_SERIAL_RX_PIN          Y_SERIAL_TX_PIN

// Z Axis
#define Z_STEP_PIN                          PB3   // PA11  needs updated
#define Z_DIR_PIN                           PB4   // PA12  needs updated
#define Z_ENABLE_PIN                        -1
#define Z_SERIAL_TX_PIN                     PA10
#define Z_SERIAL_RX_PIN          Z_SERIAL_TX_PIN

// E0 Extruder
#define E0_STEP_PIN                         PB3
#define E0_DIR_PIN                          PB4
#define E0_ENABLE_PIN                       -1
#define E0_SERIAL_TX_PIN                    PA15
#define E0_SERIAL_RX_PIN        E0_SERIAL_TX_PIN

// ---------------------------
// Heaters
// ---------------------------
#define HEATER_0_PIN                        PB6
#define HEATER_BED_PIN                      PB5

// ---------------------------
// Fans
// ---------------------------
#define FAN0_PIN                            PB7
#define FAN1_PIN                            PB8
#define FAN2_PIN                            PB9

// ---------------------------
// SD Card Configuration (Disabled)
// ---------------------------
#define SD_DETECT_PIN                       -1
#define SDCARD_CONNECTION                   -1
