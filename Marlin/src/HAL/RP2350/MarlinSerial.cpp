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
#include "../platforms.h"

#ifdef __PLAT_RP2350__

#include "../../inc/MarlinConfig.h"
#include "MarlinSerial.h"

#if ENABLED(EMERGENCY_PARSER)
  #include "../../feature/e_parser.h"
#endif

// Wrap the serial ports for Marlin
DefaultSerial0 MSerial0(false, _MARLIN_UART_0);  // Marlin Serial0 = UART0
DefaultSerial1 MSerial1(false, _MARLIN_UART_1);  // Marlin Serial1 = UART1
DefaultSerial2 MSerial2(false, Serial);          // Marlin Serial2 = USB (-1)
USBSerialType USBSerial(false, Serial);

// Called by the Arduino-Pico core before setup(). Choose the UART pins before any begin().
void initVariant() {
  #if PINS_EXIST(SERIAL0_TX, SERIAL0_RX)
    _MARLIN_UART_0.setPinout(SERIAL0_TX_PIN, SERIAL0_RX_PIN);
  #endif
  #if PINS_EXIST(SERIAL1_TX, SERIAL1_RX)
    _MARLIN_UART_1.setPinout(SERIAL1_TX_PIN, SERIAL1_RX_PIN);
  #endif
}

#endif // __PLAT_RP2350__
