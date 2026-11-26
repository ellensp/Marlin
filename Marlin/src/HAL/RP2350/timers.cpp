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

#include <hardware/clocks.h>
#include <hardware/structs/ticks.h>
#include <hardware/sync.h>

#ifdef XOSC_HZ
  static_assert((XOSC_HZ) % (HAL_TIMER_RATE) == 0, "HAL_TIMER_RATE must divide the crystal frequency evenly.");
#endif

volatile hal_timer_t HAL_timer_base[NUM_ALARMS] = { 0 },
                     HAL_timer_compare[NUM_ALARMS] = { 0 };

/**
 * Common alarm interrupt entry. Acknowledge the alarm, then restart the
 * period from the match time as an auto-reload hardware timer would.
 */
FORCE_INLINE static void HAL_timer_irq(const uint8_t timer_num) {
  hw_clear_bits(&HAL_TIMER_HW->intf, _BV(timer_num));
  HAL_TIMER_HW->intr = _BV(timer_num);

  const hal_timer_t period = HAL_timer_compare[timer_num],
                    now = HAL_TIMER_HW->timerawl;
  hal_timer_t base = HAL_timer_base[timer_num] + period;
  if (hal_timer_t(now - base) >= period) base = now;  // A whole period late? Drop the missed periods.
  HAL_timer_base[timer_num] = base;

  HAL_timer_set_compare(timer_num, period);
}

#define _HAL_TIMER_CALLBACK(T) HAL_timer_##T##_callback()
#define HAL_TIMER_CALLBACK(T) _HAL_TIMER_CALLBACK(T)

static void HAL_timer_step_irq() { HAL_timer_irq(MF_TIMER_STEP); HAL_TIMER_CALLBACK(MF_TIMER_STEP); }
static void HAL_timer_temp_irq() { HAL_timer_irq(MF_TIMER_TEMP); HAL_TIMER_CALLBACK(MF_TIMER_TEMP); }

static void HAL_timer_claim(const uint8_t timer_num, const irq_handler_t handler, const uint8_t priority) {
  timer_hardware_alarm_claim(HAL_TIMER_HW, timer_num);
  irq_set_exclusive_handler(HAL_TIMER_IRQ(timer_num), handler);
  irq_set_priority(HAL_TIMER_IRQ(timer_num), priority);
}

void HAL_timer_init() {
  // Run TIMER1 at HAL_TIMER_RATE. The SDK started its tick generator at 1MHz.
  ticks_slice_hw_t &tick = ticks_hw->ticks[TICK_TIMER1];
  tick.ctrl = 0;
  tick.cycles = clock_get_hz(clk_ref) / (HAL_TIMER_RATE);
  tick.ctrl = TICKS_TIMER1_CTRL_ENABLE_BITS;

  HAL_timer_claim(MF_TIMER_STEP, HAL_timer_step_irq, STEP_TIMER_IRQ_PRIO);
  HAL_timer_claim(MF_TIMER_TEMP, HAL_timer_temp_irq, TEMP_TIMER_IRQ_PRIO);
}

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {
  if (timer_num != MF_TIMER_STEP && timer_num != MF_TIMER_TEMP) return;

  const uint32_t saved = save_and_disable_interrupts();
  HAL_timer_base[timer_num] = HAL_TIMER_HW->timerawl;
  hw_set_bits(&HAL_TIMER_HW->inte, _BV(timer_num));
  HAL_timer_set_compare(timer_num, (HAL_TIMER_RATE) / frequency);
  restore_interrupts(saved);
}

void HAL_timer_stop(const uint8_t timer_num) {
  hw_clear_bits(&HAL_TIMER_HW->inte, _BV(timer_num));
  HAL_TIMER_HW->armed = _BV(timer_num);   // Write 1 to disarm
  hw_clear_bits(&HAL_TIMER_HW->intf, _BV(timer_num));
  HAL_TIMER_HW->intr = _BV(timer_num);
}

void __attribute__((weak)) HAL_timer_0_callback() {}
void __attribute__((weak)) HAL_timer_1_callback() {}
void __attribute__((weak)) HAL_timer_2_callback() {}
void __attribute__((weak)) HAL_timer_3_callback() {}

#endif // __PLAT_RP2350__
