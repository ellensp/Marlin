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
 * ThreeBee pin assignments
 * ESP32-S3-WROOM-1U-N16 (no PSRAM), native USB, three 74HC595 on the I2S stepper stream.
 *
 * Module variants with octal PSRAM (N8R8, N16R8, R16V) are not supported:
 * they use GPIO 35-37 (the driver SPI bus) internally.
 */

#include "env_validate.h"

#if EXTRUDERS > 2 || E_STEPPERS > 2
  #error "ThreeBee supports up to 2 E steppers."
#elif HOTENDS > 2
  #error "ThreeBee supports up to 2 hotends / E steppers."
#endif

#define BOARD_INFO_NAME      "ThreeBee"
#define DEFAULT_MACHINE_NAME BOARD_INFO_NAME

// MAX_EXPANDER_BITS is defined for ThreeBee in HAL/ESP32/inc/Conditionals_adv.h

//
// Servos
//
#define SERVO0_PIN                            12  // BLTouch port. 3.3V, not level-shifted

//
// Limit Switches
// The BLTouch port's trigger pin is the Z endstop input, so a probe uses Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN.
// With a DIAG jumper fitted the X, Y or Z driver's DIAG output drives the same input (sensorless homing).
//
#define X_STOP_PIN                            17  // X
#define Y_STOP_PIN                            16  // Y
#define Z_STOP_PIN                            15  // Z
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                       7  // Runout 1
#endif
#ifndef FIL_RUNOUT2_PIN
  #define FIL_RUNOUT2_PIN                      6  // Runout 2
#endif

//
// Probe enable
//
#if ENABLED(PROBE_ENABLE_DISABLE) && !defined(PROBE_ENABLE_PIN)
  #define PROBE_ENABLE_PIN            SERVO0_PIN  // BLTouch port
#endif

//
// Enable I2S stepper stream
//
#ifndef I2S_STEPPER_STREAM
  #define I2S_STEPPER_STREAM
#endif
#if ENABLED(I2S_STEPPER_STREAM)
  #define I2S_WS                              21
  #define I2S_BCK                             14
  #define I2S_DATA                             0  // Also the BOOT strap. An output, so the BOOT button only matters at reset
#endif

//
// Misc. Functions
//
#define LED_PIN                              128  // "STAT" LED, shift register bit 0

//
// Steppers
//
#define X_STEP_PIN                           134  // X
#define X_DIR_PIN                            133
#define X_ENABLE_PIN                         135

#define Y_STEP_PIN                           131  // Y
#define Y_DIR_PIN                            130
#define Y_ENABLE_PIN                         132

#define Z_STEP_PIN                           143  // Z (two motor connectors on one driver)
#define Z_DIR_PIN                            142
#define Z_ENABLE_PIN                         129

#define E0_STEP_PIN                          140  // E0
#define E0_DIR_PIN                           139
#define E0_ENABLE_PIN                        141

#define E1_STEP_PIN                          137  // E1
#define E1_DIR_PIN                           136
#define E1_ENABLE_PIN                        138

/**
 * One line per driver goes to socket pin 4 through the driver's jumper array:
 * PDN_UART on a TMC2208 / TMC2209, CS on a TMC2130 / TMC5160, MS3 on an A4988 / DRV8825.
 */
#define X_DRIVER_IO_PIN                        1
#define Y_DRIVER_IO_PIN                        2
#define Z_DRIVER_IO_PIN                       11
#define E0_DRIVER_IO_PIN                      44
#define E1_DRIVER_IO_PIN                      43

#if HAS_TMC_UART
  /**
   * TMC2208 / TMC2209 stepper drivers
   *
   * There is no multiplexer chip and there are not enough UARTs for five drivers.
   * One hardware UART (UART1) serves them all: the ESP32's GPIO matrix routes its TX and RX
   * to the single-wire pin of the driver being addressed. See HAL/ESP32/TMCMuxSerial.h.
   */
  #define TMC_UART_GPIO_MUX

  #define X_HARDWARE_SERIAL        tmcMuxSerialX
  #define Y_HARDWARE_SERIAL        tmcMuxSerialY
  #define Z_HARDWARE_SERIAL        tmcMuxSerialZ
  #define E0_HARDWARE_SERIAL      tmcMuxSerialE0
  #define E1_HARDWARE_SERIAL      tmcMuxSerialE1

  #define X_SERIAL_TX_PIN        X_DRIVER_IO_PIN
  #define X_SERIAL_RX_PIN        X_SERIAL_TX_PIN
  #define Y_SERIAL_TX_PIN        Y_DRIVER_IO_PIN
  #define Y_SERIAL_RX_PIN        Y_SERIAL_TX_PIN
  #define Z_SERIAL_TX_PIN        Z_DRIVER_IO_PIN
  #define Z_SERIAL_RX_PIN        Z_SERIAL_TX_PIN
  #define E0_SERIAL_TX_PIN      E0_DRIVER_IO_PIN
  #define E0_SERIAL_RX_PIN      E0_SERIAL_TX_PIN
  #define E1_SERIAL_TX_PIN      E1_DRIVER_IO_PIN
  #define E1_SERIAL_RX_PIN      E1_SERIAL_TX_PIN

  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                 115200
  #endif
#endif

#if HAS_TMC_SPI
  /**
   * TMC2130 / TMC5160 stepper drivers
   * Software SPI on a bus of its own, behind the "Driver SPI" isolation jumpers.
   */
  #ifndef TMC_USE_SW_SPI
    #define TMC_USE_SW_SPI
  #endif
  #define TMC_SPI_SCK                         35
  #define TMC_SPI_MOSI                        36
  #define TMC_SPI_MISO                        37

  #define X_CS_PIN               X_DRIVER_IO_PIN
  #define Y_CS_PIN               Y_DRIVER_IO_PIN
  #define Z_CS_PIN               Z_DRIVER_IO_PIN
  #define E0_CS_PIN             E0_DRIVER_IO_PIN
  #define E1_CS_PIN             E1_DRIVER_IO_PIN
#endif

//
// Temperature Sensors
// TH1, TH2 and TB are read by an ADS1115 on I2C, not by the module's ADC, so there are no TEMP_n_PIN.
// Required settings:
//   ADS1115_THERMISTORS
//   ADS1115_ADDRESS       0x48
//   ADS1115_VREF_MV       3300
//   ADS1115_VREF_CHANNEL     2  // the 3.3V rail
//   ADS1115_CHANNEL_0        0  // TH1
//   ADS1115_CHANNEL_1        1  // TH2
//   ADS1115_CHANNEL_BED      3  // TB
//
#define I2C_SDA_PIN                            5
#define I2C_SCL_PIN                            4

//
// Heaters / Fans
//
#define HEATER_0_PIN                         148  // HE0
#define HEATER_1_PIN                         146  // HE1
#define HEATER_BED_PIN                       145  // H-BED
#define FAN0_PIN                             149  // FAN1
#define FAN1_PIN                             150  // FAN2
#define FAN2_PIN                             151  // FAN3
#define FAN3_PIN                             147  // FAN4

/**
 *                 ------                                 ------
 *  (BEEPER)   45 | 1  2 | 10  (BTN_ENC)   (SPI MISO) 41 | 1  2 | 40 (SPI SCK)
 *  (LCD_EN)    9 | 3  4 | 46  (LCD_RS)     (BTN_EN1)  8 | 3  4 | 38 (SPI CS)
 *  (LCD_D4)    3 | 5  6   48  (LCD_D5)     (BTN_EN2) 18 | 5  6   39 (SPI MOSI)
 *  (LCD_D6)   47 | 7  8 | 13  (LCD_D7)     (SPI_DET) 42 | 7  8 | RESET
 *            GND | 9 10 | 5V                        GND | 9 10 | 3.3V
 *                 ------                                 ------
 *                  EXP1                                   EXP2
 *
 * All EXP pins are 3.3V GPIO, not level-shifted, and can be inputs or outputs.
 * GPIO 45 and 46 are strapping pins: nothing on them may pull them high at reset.
 */

#define EXP1_01_PIN                           45
#define EXP1_02_PIN                           10
#define EXP1_03_PIN                            9
#define EXP1_04_PIN                           46
#define EXP1_05_PIN                            3
#define EXP1_06_PIN                           48
#define EXP1_07_PIN                           47
#define EXP1_08_PIN                           13

#define EXP2_01_PIN                           41
#define EXP2_02_PIN                           40
#define EXP2_03_PIN                            8
#define EXP2_04_PIN                           38
#define EXP2_05_PIN                           18
#define EXP2_06_PIN                           39
#define EXP2_07_PIN                           42
#define EXP2_08_PIN                           -1  // RESET

#define EXP_REVERSE_KEYED

//
// Serial display header (5V GND TX RX)
// TX is EXP1_08 (GPIO 13) and RX is EXP1_06 (GPIO 48), so it is used instead of EXP1.
//

//
// MicroSD card
// The onboard socket and EXP2 share one SPI bus, chip select and card detect.
//
#define SD_MOSI_PIN                  EXP2_06_PIN
#define SD_MISO_PIN                  EXP2_01_PIN
#define SD_SCK_PIN                   EXP2_02_PIN
#define SD_SS_PIN                    EXP2_04_PIN
#define SD_DETECT_PIN                EXP2_07_PIN

#if HAS_WIRED_LCD
  #define BEEPER_PIN                 EXP1_01_PIN
  #define LCD_PINS_EN                EXP1_03_PIN
  #define LCD_PINS_RS                EXP1_04_PIN
  #define BTN_ENC                    EXP1_02_PIN
  #define BTN_EN1                    EXP2_03_PIN
  #define BTN_EN2                    EXP2_05_PIN
  #define LCD_BACKLIGHT_PIN                   -1

  #if ENABLED(MKS_MINI_12864)
    // MKS MINI12864 and MKS LCD12864B; If using MKS LCD12864A (Need to remove RPK2 resistor)
    #define DOGLCD_CS                EXP1_06_PIN
    #define DOGLCD_A0                EXP1_07_PIN
    #define LCD_RESET_PIN                     -1
  #elif ENABLED(FYSETC_MINI_12864_2_1)
    // MKS_MINI_12864_V3, BTT_MINI_12864, FYSETC_MINI_12864_2_1, BEEZ_MINI_12864
    #define DOGLCD_CS                EXP1_03_PIN
    #define DOGLCD_A0                EXP1_04_PIN
    #define LCD_RESET_PIN            EXP1_05_PIN
    #define NEOPIXEL_PIN             EXP1_06_PIN
    #if SD_CONNECTION_IS(ONBOARD)
      #define FORCE_SOFT_SPI
    #endif
    #if ALL(MKS_MINI_12864_V3, HAS_MEDIA)
      #define PAUSE_LCD_FOR_BUSY_SD
    #endif
  #else
    #define LCD_PINS_D4              EXP1_05_PIN
    #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
      #define LCD_PINS_D5            EXP1_06_PIN
      #define LCD_PINS_D6            EXP1_07_PIN
      #define LCD_PINS_D7            EXP1_08_PIN
    #endif
    #define BOARD_ST7920_DELAY_1              96
    #define BOARD_ST7920_DELAY_2              48
    #define BOARD_ST7920_DELAY_3             600
  #endif
#endif // HAS_WIRED_LCD
