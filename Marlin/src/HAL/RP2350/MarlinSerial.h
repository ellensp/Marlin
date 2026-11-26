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

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(EMERGENCY_PARSER)
  #include "../../feature/e_parser.h"
#endif

#include "../../core/serial_hook.h"

/**
 * Serial Port Configuration for RP2350 (Raspberry Pi Pico 2)
 *
 *   SERIAL_PORT   Arduino-Pico object      Marlin wrapper
 *       -1        Serial  (USB CDC)        MSerial2, USBSerial
 *        0        Serial1 (UART0)          MSerial0
 *        1        Serial2 (UART1)          MSerial1
 *
 * The core's own Serial1 and Serial2 objects are used, since its UART interrupt
 * handlers call them by name. Their pins are chosen in initVariant() (MarlinSerial.cpp)
 * from SERIAL0_TX_PIN / SERIAL0_RX_PIN and SERIAL1_TX_PIN / SERIAL1_RX_PIN.
 */
#define _MARLIN_UART_0 Serial1
#define _MARLIN_UART_1 Serial2

typedef ForwardSerial1Class< decltype(_MARLIN_UART_0) > DefaultSerial0;
extern DefaultSerial0 MSerial0;
typedef ForwardSerial1Class< decltype(_MARLIN_UART_1) > DefaultSerial1;
extern DefaultSerial1 MSerial1;
typedef ForwardSerial1Class< decltype(Serial) > DefaultSerial2;
extern DefaultSerial2 MSerial2;
typedef ForwardSerial1Class< decltype(Serial) > USBSerialType;
extern USBSerialType USBSerial;

#define _DECLARE_SERIAL(X) \
  typedef ForwardSerial1Class<decltype(_MARLIN_UART_##X)> DefaultSerial##X; \
  extern DefaultSerial##X MSerial##X
#define DECLARE_SERIAL(X) _DECLARE_SERIAL(X)

#define SERIAL_INDEX_MIN 0
#define SERIAL_INDEX_MAX 1
#define USB_SERIAL_PORT(...) MSerial2
#include "../shared/serial_ports.h"

#if defined(LCD_SERIAL_PORT) && ANY(HAS_DGUS_LCD, EXTENSIBLE_UI)
  #define SERIAL_GET_TX_BUFFER_FREE() LCD_SERIAL.availableForWrite()
#endif
