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
 * TMC_UART_GPIO_MUX
 *
 * One hardware UART for several TMC2208 / TMC2209 drivers that each have a GPIO of their own,
 * wired to the driver's single-wire PDN_UART pin. There is no multiplexer chip: the ESP32's
 * GPIO matrix routes the UART's TX and RX to the pin of the driver being addressed.
 *
 * A pin is an open-drain output with a pull-up while its driver is addressed, so the UART
 * never drives against a replying driver. All other pins are inputs with a pull-up.
 *
 * HAL.h is included before the pins file that enables this, so the declarations are unconditional.
 */

#include <Stream.h>

class TMCMuxSerial : public Stream {
  public:
    TMCMuxSerial(const int8_t pin) : pin(pin) {}

    void begin(const uint32_t baud);

    int available() override;
    int read() override;
    int peek() override;
    size_t write(uint8_t c) override;
    void flush() override;
    using Print::write;

  private:
    const int8_t pin;       // This driver's PDN_UART pin
    static int8_t active;   // The pin the UART is routed to, -1 for none
    void select();
};

// One per axis with a driver on the shared UART. A pins file selects them with, e.g.,
//   #define X_HARDWARE_SERIAL tmcMuxSerialX
//   #define X_SERIAL_TX_PIN   <pin>
extern TMCMuxSerial tmcMuxSerialX, tmcMuxSerialX2, tmcMuxSerialY, tmcMuxSerialY2,
                    tmcMuxSerialZ, tmcMuxSerialZ2, tmcMuxSerialZ3, tmcMuxSerialZ4,
                    tmcMuxSerialI, tmcMuxSerialJ, tmcMuxSerialK,
                    tmcMuxSerialU, tmcMuxSerialV, tmcMuxSerialW,
                    tmcMuxSerialE0, tmcMuxSerialE1, tmcMuxSerialE2, tmcMuxSerialE3,
                    tmcMuxSerialE4, tmcMuxSerialE5, tmcMuxSerialE6, tmcMuxSerialE7;
