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
#ifdef ARDUINO_ARCH_ESP32

#include "../../inc/MarlinConfig.h"

#if ENABLED(TMC_UART_GPIO_MUX)

#include "TMCMuxSerial.h"

#include <HardwareSerial.h>
#include <driver/gpio.h>
#include <soc/gpio_sig_map.h>
#include <esp32-hal-matrix.h>

#define MUX_UART Serial1

int8_t TMCMuxSerial::active = -1;

static void idle_pin(const int8_t pin) {
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = 1ULL << pin;
  cfg.mode = GPIO_MODE_INPUT;
  cfg.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&cfg);
}

void TMCMuxSerial::begin(const uint32_t baud) {
  static bool started = false;
  if (!started) {
    started = true;
    // Give the UART this pin. Without pins it would take its default ones.
    MUX_UART.begin(baud, SERIAL_8N1, pin, pin);
    pinMatrixOutDetach(pin, false, false);
  }
  idle_pin(pin);
}

/**
 * Route the UART to this driver's pin.
 * Only this pin and the previously selected one are touched.
 */
void TMCMuxSerial::select() {
  if (active == pin) return;

  MUX_UART.flush();                               // The last byte must leave the shifter first (TX only)

  if (active >= 0) {
    pinMatrixOutDetach(active, false, false);
    gpio_set_direction(gpio_num_t(active), GPIO_MODE_INPUT);
  }

  // Open-drain and high, so the line stays released until the UART takes it over
  gpio_set_level(gpio_num_t(pin), 1);
  gpio_set_direction(gpio_num_t(pin), GPIO_MODE_INPUT_OUTPUT_OD);
  pinMatrixOutAttach(pin, U1TXD_OUT_IDX, false, false);
  pinMatrixInAttach(pin, U1RXD_IN_IDX, false);

  while (MUX_UART.available()) MUX_UART.read();   // Nothing received so far belongs to this driver

  active = pin;
}

int TMCMuxSerial::available()       { select(); return MUX_UART.available(); }
int TMCMuxSerial::read()            { select(); return MUX_UART.read(); }
int TMCMuxSerial::peek()            { select(); return MUX_UART.peek(); }
size_t TMCMuxSerial::write(uint8_t c) { select(); return MUX_UART.write(c); }
void TMCMuxSerial::flush()          { MUX_UART.flush(); }

#if AXIS_HAS_HW_SERIAL(X)
  TMCMuxSerial tmcMuxSerialX(X_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(X2)
  TMCMuxSerial tmcMuxSerialX2(X2_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(Y)
  TMCMuxSerial tmcMuxSerialY(Y_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(Y2)
  TMCMuxSerial tmcMuxSerialY2(Y2_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(Z)
  TMCMuxSerial tmcMuxSerialZ(Z_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(Z2)
  TMCMuxSerial tmcMuxSerialZ2(Z2_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(Z3)
  TMCMuxSerial tmcMuxSerialZ3(Z3_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(Z4)
  TMCMuxSerial tmcMuxSerialZ4(Z4_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(I)
  TMCMuxSerial tmcMuxSerialI(I_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(J)
  TMCMuxSerial tmcMuxSerialJ(J_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(K)
  TMCMuxSerial tmcMuxSerialK(K_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(U)
  TMCMuxSerial tmcMuxSerialU(U_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(V)
  TMCMuxSerial tmcMuxSerialV(V_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(W)
  TMCMuxSerial tmcMuxSerialW(W_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(E0)
  TMCMuxSerial tmcMuxSerialE0(E0_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(E1)
  TMCMuxSerial tmcMuxSerialE1(E1_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(E2)
  TMCMuxSerial tmcMuxSerialE2(E2_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(E3)
  TMCMuxSerial tmcMuxSerialE3(E3_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(E4)
  TMCMuxSerial tmcMuxSerialE4(E4_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(E5)
  TMCMuxSerial tmcMuxSerialE5(E5_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(E6)
  TMCMuxSerial tmcMuxSerialE6(E6_SERIAL_TX_PIN);
#endif
#if AXIS_HAS_HW_SERIAL(E7)
  TMCMuxSerial tmcMuxSerialE7(E7_SERIAL_TX_PIN);
#endif

#endif // TMC_UART_GPIO_MUX
#endif // ARDUINO_ARCH_ESP32
