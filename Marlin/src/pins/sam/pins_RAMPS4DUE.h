/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * Arduino Mega or Due with RAMPS4DUE pin assignments
 *
 * Applies to the following boards:
 *
 *  RAMPS4DUE_EFB (Hotend, Fan, Bed)
 *  RAMPS4DUE_EEB (Hotend0, Hotend1, Bed)
 *  RAMPS4DUE_EFF (Hotend, Fan0, Fan1)
 *  RAMPS4DUE_EEF (Hotend0, Hotend1, Fan)
 *  RAMPS4DUE_SF  (Spindle, Controller Fan)
 *
 *  Differences between
 *  RAMPS_14 | RAMPS4DUE
 *       A13 | A9/D63 (shares the same pin with AUX2_4PIN)
 *       A14 | A10/D64 (shares the same pin with AUX2_5PIN)
 *       A15 | NC
 */

#define ALLOW_SAM3X8E
#define BOARD_INFO_NAME "RAMPS4DUE"

//
// Temperature Sensors
//
#define TEMP_0_PIN                             9  // Analog Input
#define TEMP_1_PIN                            -1  // Analog Input
#define TEMP_BED_PIN                          10  // Analog Input

#if HAS_SPI_TFT
  #define LCD_PINS_DEFINED
#endif

#include "../ramps/pins_RAMPS.h"

#if HAS_SPI_TFT
  #define LCD_BACKLIGHT_PIN          EXP1_03_PIN

  //#define TOUCH_CS_PIN               EXP1_05_PIN  // SPI1_NSS
  //#define TOUCH_SCK_PIN              EXP2_02_PIN  // SPI1_SCK
  //#define TOUCH_MISO_PIN             EXP2_01_PIN  // SPI1_MISO
  //#define TOUCH_MOSI_PIN             EXP2_06_PIN  // SPI1_MOSI

  //#define BTN_EN1                    EXP2_03_PIN
  //#define BTN_EN2                    EXP2_05_PIN
  //#define BTN_ENC                    EXP1_02_PIN

  #define TFT_CS_PIN                 EXP1_07_PIN
  #define TFT_SCK_PIN                EXP2_02_PIN
  #define TFT_MISO_PIN               EXP2_01_PIN
  #define TFT_MOSI_PIN               EXP2_06_PIN
  #define TFT_DC_PIN                 EXP1_08_PIN
  #define TFT_A0_PIN                  TFT_DC_PIN

  #define TFT_RESET_PIN              EXP1_04_PIN
  #define TFT_BACKLIGHT_PIN    LCD_BACKLIGHT_PIN

  //#define TOUCH_BUTTONS_HW_SPI
  #define TOUCH_BUTTONS_HW_SPI_DEVICE          1

  #define LCD_USE_DMA_SPI

#endif

