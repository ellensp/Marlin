/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2026 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * Adafruit HUZZAH32 ESP32 Feather pin assignments
 * https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/pinouts
 *
 * Stepper drivers (e.g., A4988, DRV8825, TMC2208 standalone) are wired straight
 * to the Feather's header pins. Only X and Y are assigned, as for a plotter.
 *
 * Avoided: the strapping pins (GPIO 0, 2, 5, 12, 15), except GPIO 5 as SPI SCK,
 * the USB serial pins (GPIO 1, 3) and GPIO 35, the battery voltage divider.
 * Still free: GPIO 4 (A5), 13 (red LED), 16 (RX), 17 (TX), 22 (SCL), 23 (SDA)
 * and the input-only GPIO 36 (A4).
 */

#include "env_validate.h"

#define BOARD_INFO_NAME "Adafruit Feather ESP32"

//
// Limit Switches
//
// GPIO 34 and 39 are input-only with no internal pull-ups, so the switches need external pull-up resistors
#define X_STOP_PIN                            34  // A2
#define Y_STOP_PIN                            39  // A3

//
// Steppers
//
#define X_STEP_PIN                            26  // A0
#define X_DIR_PIN                             25  // A1
#define X_ENABLE_PIN                          32

#define Y_STEP_PIN                            27
#define Y_DIR_PIN                             33
#define Y_ENABLE_PIN                          32  // Shared with X

//
// Servos
//
#define SERVO0_PIN                            14  // e.g., pen lift

//
// SD Card on the SPI header pins
//
#define SD_SCK_PIN                             5  // SCK
#define SD_MOSI_PIN                           18  // MO
#define SD_MISO_PIN                           19  // MI
#define SD_SS_PIN                             21
