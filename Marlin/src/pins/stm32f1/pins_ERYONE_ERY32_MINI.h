/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * Eryone Ery32 mini (STM32F103VET6) board pin assignments
 */

#include "env_validate.h"

#if HOTENDS > 1 || E_STEPPERS > 1
  #error "Eryone Ery32 mini supports up to 1 hotends / E steppers."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "ERYONE Ery32 mini"
#endif

//#define DISABLE_DEBUG
#define DISABLE_JTAG
//#define ENABLE_SPI3

#if ANY(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE     (0x800U)           // 2K
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE (EEPROM_PAGE_SIZE)
#endif

//
// Servos
//
#define SERVO0_PIN                          PA12

//
// Limit Switches
//
#define X_STOP_PIN                          PD8
#define Y_STOP_PIN                          PD15
#define Z_STOP_PIN                          PA9
#define Z_MIN_PROBE_PIN                     PA11

//
// Steppers
//
#define X_STEP_PIN                          PB15
#define X_DIR_PIN                           PB14
#define X_ENABLE_PIN                        PD10

#define Y_STEP_PIN                          PD14
#define Y_DIR_PIN                           PD13
#define Y_ENABLE_PIN                        PC6

#define Z_STEP_PIN                          PC8
#define Z_DIR_PIN                           PC7
#define Z_ENABLE_PIN                        PA8

#define E0_STEP_PIN                         PE13
#define E0_DIR_PIN                          PE14
#define E0_ENABLE_PIN                       PB13

//
// Heaters 0,1 / Fans / Bed
//
#define HEATER_0_PIN                        PD11
#define HEATER_BED_PIN                      PD12
#ifndef FAN0_PIN
  #define FAN0_PIN                          PB5
#endif
#define FAN1_PIN                            PB4
#define FAN2_PIN                            PB9

#define FAN_SOFT_PWM_REQUIRED

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  #define X_HARDWARE_SERIAL  MSerial4
  #define Y_HARDWARE_SERIAL  MSerial4
  #define Z_HARDWARE_SERIAL  MSerial4
  #define E0_HARDWARE_SERIAL MSerial4

  // Default TMC slave addresses
  #ifndef X_SLAVE_ADDRESS
    #define X_SLAVE_ADDRESS                    2
  #endif
  #ifndef Y_SLAVE_ADDRESS
    #define Y_SLAVE_ADDRESS                    3
  #endif
  #ifndef Z_SLAVE_ADDRESS
    #define Z_SLAVE_ADDRESS                    1
  #endif
  #ifndef E0_SLAVE_ADDRESS
    #define E0_SLAVE_ADDRESS                   0
  #endif
  static_assert(X_SLAVE_ADDRESS == 2, "X_SLAVE_ADDRESS must be 2 for BOARD_ERYONE_ERY32_MINI.");
  static_assert(Y_SLAVE_ADDRESS == 3, "Y_SLAVE_ADDRESS must be 3 for BOARD_ERYONE_ERY32_MINI.");
  static_assert(Z_SLAVE_ADDRESS == 1, "Z_SLAVE_ADDRESS must be 1 for BOARD_ERYONE_ERY32_MINI.");
  static_assert(E0_SLAVE_ADDRESS == 0, "E0_SLAVE_ADDRESS must be 0 for BOARD_ERYONE_ERY32_MINI.");
#endif

//
// Temperature Sensors
//
#define TEMP_BED_PIN                        PC2   // TB
#define TEMP_0_PIN                          PC1   // TH1
#define FIL_RUNOUT_PIN                      PA10  // MT_DET

#ifndef TEMP_BOARD_PIN
  #define TEMP_BOARD_PIN                    PC3   // TH2
#endif
#if TEMP_BOARD_PIN == PC3 && TEMP_SENSOR_BOARD != 13
  #warning "The built-in TEMP_SENSOR_BOARD is 13 for ERYONE Ery32 mini."
#endif

//
// LCD Pins
//

/**
 *              ------                                  ------
 *(BEEPER) PE12 |1  2| PE11 (BTN_ENC)        (MISO) PA6 |1  2| PA5 (SCK)
 *(LCD_EN) PE10 |3  4| PE9   (LCD_RS)     (BTN_EN1) PE4 |3  4|     (SD_SS)
 *(LCD_D4) PE8   5  6| PE7   (LCD_D5)     (BTN_EN2) PE3  5  6| PA7 (MOSI)
 *(LCD_D6) PB2  |7  8| PB1   (LCD_D7)   (SD_DETECT)     |7  8|     (RESET)
 *         GND  |9 10| VCC                          GND |9 10|
 *              ------                                  ------
 *               Exp1                                    Exp2
 */

#if HAS_WIRED_LCD
  #define BEEPER_PIN                        PE12
  #define BTN_ENC                           PE11
  #define LCD_PINS_EN                       PE10
  #define LCD_PINS_RS                       PE9
  #define BTN_EN1                           PE4
  #define BTN_EN2                           PE3
  #define LCD_PINS_D4                       PE8
  #define LCD_PINS_D5                       PE7
  #define LCD_PINS_D6                       PB2
  #define LCD_PINS_D7                       PB1

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define BTN_ENC_EN               LCD_PINS_D7  // Detect the presence of the encoder
  #endif

  #define BOARD_ST7920_DELAY_1                50
  #define BOARD_ST7920_DELAY_2                50
  #define BOARD_ST7920_DELAY_3                50

#endif // HAS_WIRED_LCD

//
// SD Card
//
#define SDCARD_CONNECTION ONBOARD
#define ENABLE_SPI1
#define SD_DETECT_PIN                       PA4
#define SCK_PIN                             PA5
#define MISO_PIN                            PA6
#define MOSI_PIN                            PA7
#define SS_PIN                              PC4

// STM32F103VET6  | DIGITAL       | ANALOG        | USART      | TWI       | SPI                  | SPECIAL    |
//----------------|---------------|---------------|------------|-----------|----------------------|------------|
//      PA0  0    |               |               |            |           |                      |            |
//      PA1  1    |               |               |            |           |                      |            |
//      PA2  2    |               |               | USART2_TX  |           |                      |            |
//      PA3  3    |               |               | USART2_RX  |           |                      |            |
//      PA4  4    | I SD_DETECT   |               |            |           |                      |            |
//      PA5  5    | SD_CARD       |               |            |           | SPI1_SCK             |            |
//      PA6  6    | SD_CARD       |               |            |           | SPI1_MISO            |            |
//      PA7  7    | SD_CARD       |               |            |           | SPI1_MOSI            |            |
//      PA8  8    | O  Z_ENABLE   |               |            |           |                      |            |
//      PA9  9    |               |               |            |           |                      |            |
//      PA10 10   | I FIL_RUNOUT  |               |            |           |                      |            |
//      PA11 11   | I  Z_MIN      |               |            |           |                      |            |
//      PA12 12   | O  PWM?       |               |            |           |                      | SERVO0     | sofware pwm?
//      PA13 13   |               |               |            |           |                      | SWDIO      |
//      PA14 14   |               |               |            |           |                      | SWCLK      |
//      PA15 15   |               |               |            |           |                      |            |
//                |---------------|---------------|------------|-----------|----------------------|------------|
//      PB0  16   | ? WIFI-CPI00  |               |            |           |                      |            |
//      PB1  17   | O  LCD_PINS_D7|               |            |           |                      |            |
//      PB2  18   | O  LCD_PINS_D6|               |            |           |                      |            |
//      PB3  19   |               |               |            |           | SPI3_SCK             |            |
//      PB4  20   | O  FAN1       |               |            |           | SPI3_MISO            |            |
//      PB5  21   | O  FAN0       |               |            |           | SPI3_MOSI            |            |
//      PB6  22   |               |               |            |           |                      |            |
//      PB7  23   |               |               |            |           |                      |            |
//      PB8  24   |               |               |            |           |                      |            |
//      PB9  25   | O FAN2        |               |            |           |                      |            |
//      PB10 26   |               |               | USART3_TX  |           |                      |            |
//      PB11 27   |               |               | USART3_RX  |           |                      |            |
//      PB12 28   |               |               |            |           |                      |            |
//      PB13 29   | O  E0_ENABLE  |               |            |           |                      |            |
//      PB14 30   | O  X_DIR      |               |            |           |                      |            |
//      PB15 31   | O  X_STOP     |               |            |           |                      |            |
//                |---------------|---------------|------------|-----------|----------------------|------------|
//      PC0  32   |               |               |            |           |                      |            |
//      PC1  33   |               | A1 Nozzle T°c |            |           |                      |            |
//      PC2  34   |               | A0 Bed T°c    |            |           |                      |            |
//      PC3  35   |               | A3 Board T°c  |            |           |                      |            |
//      PC4  36   | SD_CARD       |               |            |           | SS                   |            |
//      PC5  37   |               |               |            |           |                      |            |
//      PC6  38   | O  Y_ENABLE   |               |            |           |                      |            |
//      PC7  39   | O  Z_DIR      |               |            |           |                      |            |
//      PC8  40   | O  Z_STEP     |               |            |           |                      |            |
//      PC9  41   |               |               |            |           |                      |            |
//      PC10 42   | SPI UART      |               | UART4_TX   |           |                      |            |
//      PC11 43   | SPI UART      |               | UART4_RX   |           |                      |            |
//      PC12 44   |               |               |            |           |                      |            |
//      PC13 45   |               |               |            |           |                      |            |
//      PC14 46   |               |               |            |           |                      |            |
//      PC15 47   |               |               |            |           |                      |            |
//                |---------------|---------------|------------|-----------|----------------------|------------|
//      PD0  48   |               |               |            |           |                      |            |
//      PD1  49   |               |               |            |           |                      |            |
//      PD2  50   |               |               |            |           |                      |            |
//      PD3  51   |               |               |            |           |                      |            |
//      PD4  52   |               |               |            |           |                      |            |
//      PD5  53   |               |               |            |           |                      |            |
//      PD6  54   |               |               |            |           |                      |            |
//      PD7  55   |               |               |            |           |                      |            |
//      PD8  56   | I  X_STOP     |               |            |           |                      |            |
//      PD9  57   |               |               |            |           |                      |            |
//      PD10 58   | O  X_ENABLE   |               |            |           |                      |            |
//      PD11 59   | O  NOZZLE     |               |            |           |                      |            |
//      PD12 60   | O  BED        |               |            |           |                      |            |
//      PD13 61   | O  Y_DIR      |               |            |           |                      |            |
//      PD14 62   | O  Y_STEP     |               |            |           |                      |            |
//      PD15 63   | I  Y_STOP     |               |            |           |                      |            |
//                |---------------|---------------|------------|-----------|----------------------|------------|
//      PE0  64   |               |               |            |           |                      |            |
//      PE1  65   |               |               |            |           |                      |            |
//      PE2  66   |               |               |            |           |                      |            |
//      PE3  67   | I  BTN_EN2    |               |            |           |                      |            |
//      PE4  68   | I  BTN_EN1    |               |            |           |                      |            |
//      PE5  69   |               |               |            |           |                      |            |
//      PE6  70   |               |               |            |           |                      |            |
//      PE7  71   | O  LCD_PINS_D5|               |            |           |                      |            |
//      PE8  72   | O  LCD_PINS_D4|               |            |           |                      |            |
//      PE9  73   | O  LCD_PINS_RS|               |            |           |                      |            |
//      PE10 74   | O  LCD_PINS_EN|               |            |           |                      |            |
//      PE11 75   | I  BTN_ENC    |               |            |           |                      |            |
//      PE12 76   | O  BEEPER     |               |            |           |                      |            |
//      PE13 77   | O  E0_STEP    |               |            |           |                      |            |
//      PE14 78   | O  E0_DIR     |               |            |           |                      |            |
//      PE15 79   |               |               |            |           |                      |            |
//----------------|---------------|---------------|------------|-----------|----------------------|------------|

/* NOTES to check
#define PIN_SERIAL_RX                       PA10
#define PIN_SERIAL_TX                       PA9

#ifndef TIMER_TONE
  #define TIMER_TONE            TIM6  // TIMER_TONE must be defined in this file
#endif
#ifndef TIMER_SERVO
  #define TIMER_SERVO           TIM7  // TIMER_SERVO must be defined in this file
#endif

// I2C Definitions
#define PIN_WIRE_SDA                        PB7
#define PIN_WIRE_SCL                        PB6

#define LED_BUILTIN                         PB11

// Extra HAL modules
#if defined(STM32F103xE) || defined(STM32F103xG)
#define HAL_DAC_MODULE_ENABLED
#define HAL_SD_MODULE_ENABLED
#define HAL_SRAM_MODULE_ENABLED
#endif

                     -DSS_TIMER=4
                      -DTIMER_SERVO=TIM5
                       -DMCU_STM32F103VE
*/
