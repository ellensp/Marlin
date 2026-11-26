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

#include "../../inc/MarlinConfigPre.h"

#include "HAL.h"

#include <hardware/pwm.h>
#include <hardware/clocks.h>

/**
 * PWM on the RP2350's hardware slices, without the Arduino core's analogWrite,
 * which uses one frequency for every pin.
 *
 * Each slice has its own clock divider and period, and two outputs (A and B).
 * GPIO n is on slice (n / 2) % 8 (GPIO 32-47 on the RP2350B use slices 8-11),
 * so a frequency applies to the pins that share a slice, e.g., GP14 and GP15,
 * and also GP30 and GP31 on the RP2350B.
 */

static uint16_t pwm_slice_started = 0;  // Slices already given a frequency

static void pwm_slice_set_frequency(const uint slice, const uint32_t f_desired) {
  if (!f_desired) return;

  // The divider is 8.4 fixed-point, from 1 to 255 15/16. Use the smallest one that keeps the period
  // in 16 bits, for the finest duty resolution. A period of 65535 counts leaves room for a 100% level.
  const uint64_t clk16 = uint64_t(clock_get_hz(clk_sys)) * 16;
  const uint32_t div16 = constrain((clk16 + uint64_t(f_desired) * 65535 - 1) / (uint64_t(f_desired) * 65535), 16U, 4095U),
                 period = constrain((clk16 + uint64_t(div16) * f_desired / 2) / (uint64_t(div16) * f_desired), 2U, 65535U);

  pwm_slice_hw_t &hw = pwm_hw->slice[slice];

  // Keep the duty cycle of both outputs
  uint32_t level_a = 0, level_b = 0;
  if (TEST(pwm_slice_started, slice)) {
    const uint32_t old_period = hw.top + 1, cc = hw.cc;
    level_a = (cc & 0xFFFF) * period / old_period;
    level_b = (cc >> 16) * period / old_period;
  }

  hw.div = div16;
  hw.top = period - 1;
  hw.cc = (level_b << 16) | level_a;
  pwm_set_enabled(slice, true);
  SBI(pwm_slice_started, slice);
}

void MarlinHAL::set_pwm_duty(const pin_t pin, const uint16_t v, const uint16_t v_size/*=255*/, const bool invert/*=false*/) {
  if (!v_size) return;

  const uint slice = pwm_gpio_to_slice_num(pin);
  if (!TEST(pwm_slice_started, slice)) pwm_slice_set_frequency(slice, PWM_FREQUENCY);

  // A level equal to the period is fully on
  const uint32_t period = pwm_hw->slice[slice].top + 1;
  uint32_t level = (uint32_t(_MIN(v, v_size)) * period + v_size / 2) / v_size;
  if (invert) level = period - level;

  pwm_set_gpio_level(pin, level);
  if (gpio_get_function(pin) != GPIO_FUNC_PWM) gpio_set_function(pin, GPIO_FUNC_PWM);
}

void MarlinHAL::set_pwm_frequency(const pin_t pin, const uint16_t f_desired) {
  pwm_slice_set_frequency(pwm_gpio_to_slice_num(pin), f_desired);
}

#endif // __PLAT_RP2350__
