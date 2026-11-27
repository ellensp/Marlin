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
 * Fast I/O interfaces for RP2350
 * These use GPIO register access for fast port manipulation.
 */

// ------------------------
// Public Variables
// ------------------------


// ------------------------
// Public functions
// ------------------------

#include <hardware/gpio.h>

void FastIO_init(); // Must be called before using fast io macros
#define FASTIO_INIT() FastIO_init()

// ------------------------
// Defines
// ------------------------

#define _BV32(b) (1UL << (b))

#ifndef PWM
  #define PWM OUTPUT
#endif

/**
 * WRITE, READ and TOGGLE use the SIO registers directly. The pin must already be a GPIO.
 * Unlike digitalWrite, writing to an input pin only sets its output latch. It doesn't
 * change the pin's pulls or direction.
 *
 * Open-drain outputs are emulated, as the pads have no open-drain mode. The output latch
 * stays low and the pin direction is switched: output to pull the line low, input to release it.
 */
extern uint64_t fastio_od_pins;   // Pins set up as open-drain outputs

void fastio_set_mode(const uint8_t io, const PinMode mode);   // pinMode, for a push-pull output or an input
void fastio_out_write(const uint8_t io, const bool v);        // Become a push-pull output with no glitch
void fastio_set_output(const uint8_t io);                     // Become a push-pull output. An output keeps its level.
void fastio_out_write_od(const uint8_t io, const bool v);     // Become an open-drain output

FORCE_INLINE static void fastio_write(const uint8_t io, const bool v) {
  if (fastio_od_pins & (1ULL << io)) gpio_set_dir(io, !v); else gpio_put(io, v);
}
FORCE_INLINE static void fastio_toggle(const uint8_t io) {
  if (fastio_od_pins & (1ULL << io)) gpio_set_dir(io, !gpio_is_dir_out(io)); else gpio_put(io, !gpio_get_out_level(io));
}

#define _WRITE(IO, V)           fastio_write((IO), (V))
#define _READ(IO)               gpio_get(IO)
#define _TOGGLE(IO)             fastio_toggle(IO)

#define _GET_MODE(IO)           gpio_is_dir_out(IO)
#define _SET_MODE(IO,M)         fastio_set_mode(IO, M)
#define _SET_OUTPUT(IO)         _SET_MODE(IO, OUTPUT)                             //!< Output Push Pull Mode & GPIO_NOPULL

#define WRITE(IO,V)             _WRITE(IO,V)
#define READ(IO)                _READ(IO)
#define TOGGLE(IO)              _TOGGLE(IO)

#define OUT_WRITE(IO,V)         fastio_out_write((IO), (V))
#define OUT_WRITE_OD(IO,V)      fastio_out_write_od((IO), (V))

#define SET_INPUT(IO)           _SET_MODE(IO, INPUT)                              //!< Input Floating Mode
#define SET_INPUT_PULLUP(IO)    _SET_MODE(IO, INPUT_PULLUP)                       //!< Input with Pull-up activation
#define SET_INPUT_PULLDOWN(IO)  _SET_MODE(IO, INPUT_PULLDOWN)                     //!< Input with Pull-down activation
#define SET_OUTPUT(IO)          fastio_set_output(IO)
#define SET_OUTPUT_OD(IO)       OUT_WRITE_OD(IO, LOW)
#define SET_PWM(IO)             _SET_MODE(IO, PWM)

#define IS_INPUT(IO)            !gpio_is_dir_out(IO)
#define IS_OUTPUT(IO)           gpio_is_dir_out(IO)

#define PWM_PIN(P)              true  // Every GPIO has a PWM slice output
#define NO_COMPILE_TIME_PWM

// digitalRead/Write wrappers
#define extDigitalRead(IO)    digitalRead(IO)
#define extDigitalWrite(IO,V) digitalWrite(IO,V)

#undef I2C_SDA
#define I2C_SDA_PIN PIN_WIRE0_SDA
#undef I2C_SCL
#define I2C_SCL_PIN PIN_WIRE0_SCL
