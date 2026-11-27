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

uint64_t fastio_od_pins = 0;

void FastIO_init() {}

void fastio_set_mode(const uint8_t io, const PinMode mode) {
  fastio_od_pins &= ~(1ULL << io);
  pinMode(io, mode);
}

/**
 * pinMode(OUTPUT) drives the pin low before its level can be set. To go straight to the
 * wanted level, make the pin a floating GPIO input, set the latch, then enable the output.
 */
void fastio_out_write(const uint8_t io, const bool v) {
  fastio_od_pins &= ~(1ULL << io);
  if (gpio_get_function(io) != GPIO_FUNC_SIO || !gpio_is_dir_out(io)) pinMode(io, INPUT);
  gpio_put(io, v);
  gpio_set_dir(io, true);
}

void fastio_out_write_od(const uint8_t io, const bool v) {
  pinMode(io, INPUT);   // A floating GPIO input with the latch low
  fastio_od_pins |= 1ULL << io;
  fastio_write(io, v);
}

#endif // __PLAT_RP2350__
