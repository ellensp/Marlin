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
 * HAL Timers for RP2350
 *
 * The RP2350 has two 64-bit TIMER blocks, each with four alarms and its own tick generator.
 * TIMER0 belongs to the SDK (millis, micros, delays, alarm pools). TIMER1 is unused by the
 * SDK and the Arduino core, so Marlin claims it, sets its tick to STEPPER_TIMER_RATE, and
 * drives the Stepper and Temperature ISRs directly from two of its alarms.
 *
 * An alarm fires when the low 32 bits of the free-running counter equal the alarm value.
 * To give Marlin the "counter resets on compare match" timer it expects, each timer keeps
 * the counter value of its last match as a base. The count is the time since that base
 * and the alarm is programmed at base + compare.
 */

#include <stdint.h>

#include <hardware/timer.h>
#include <hardware/irq.h>

#include "../../core/macros.h"

// ------------------------
// Defines
// ------------------------

#define HAL_TIMER_HW  timer1_hw   // The TIMER block owned by Marlin

#define __HAL_TIMER_ISR(T) extern void HAL_timer_##T##_callback()
#define _HAL_TIMER_ISR(T)  __HAL_TIMER_ISR(T)

typedef uint32_t hal_timer_t;
#define HAL_TIMER_TYPE_MAX hal_timer_t(UINT32_MAX)

#define HAL_TIMER_RATE         12000000 // (Hz) TIMER1 tick rate, one tick per clk_ref cycle (12MHz crystal)

// Timer Index is the TIMER1 alarm number (0-3)
#ifndef MF_TIMER_STEP
  #define MF_TIMER_STEP        0  // Timer Index for Stepper
#endif
#ifndef MF_TIMER_PULSE
  #define MF_TIMER_PULSE       MF_TIMER_STEP
#endif
#ifndef MF_TIMER_TEMP
  #define MF_TIMER_TEMP        1  // Timer Index for Temperature
#endif

#define HAL_TIMER_IRQ(T)       TIMER_ALARM_IRQ_NUM(HAL_TIMER_HW, T)

// Interrupt priorities. Lower is more urgent. Other interrupts use PICO_DEFAULT_IRQ_PRIORITY (0x80).
#ifndef STEP_TIMER_IRQ_PRIO
  #define STEP_TIMER_IRQ_PRIO  PICO_HIGHEST_IRQ_PRIORITY
#endif
#ifndef TEMP_TIMER_IRQ_PRIO
  #define TEMP_TIMER_IRQ_PRIO  0xC0
#endif

// The Stepper ISR is not called again until 1/HAL_STEP_TIMER_SHARE of the time it just ran
// has passed, so that lower priority interrupts and the main loop keep running.
#ifndef HAL_STEP_TIMER_SHARE
  #define HAL_STEP_TIMER_SHARE 8
#endif

#define TEMP_TIMER_RATE        HAL_TIMER_RATE
#define TEMP_TIMER_FREQUENCY   1000 // (Hz) Temperature ISR frequency

#define STEPPER_TIMER_RATE          HAL_TIMER_RATE                        // (Hz) Frequency of Stepper Timer ISR
#define STEPPER_TIMER_TICKS_PER_US  ((STEPPER_TIMER_RATE) / 1000000)      // (ticks/μs) Stepper Timer ticks per µs
#define STEPPER_TIMER_PRESCALE      ((F_CPU) / (STEPPER_TIMER_RATE))      // (cycles/tick) CPU cycles per Stepper Timer tick

#define PULSE_TIMER_RATE            STEPPER_TIMER_RATE                    // (Hz) Frequency of Pulse Timer
#define PULSE_TIMER_PRESCALE        STEPPER_TIMER_PRESCALE

#define ENABLE_STEPPER_DRIVER_INTERRUPT()   HAL_timer_enable_interrupt(MF_TIMER_STEP)
#define DISABLE_STEPPER_DRIVER_INTERRUPT()  HAL_timer_disable_interrupt(MF_TIMER_STEP)
#define STEPPER_ISR_ENABLED()               HAL_timer_interrupt_enabled(MF_TIMER_STEP)

#define ENABLE_TEMPERATURE_INTERRUPT()  HAL_timer_enable_interrupt(MF_TIMER_TEMP)
#define DISABLE_TEMPERATURE_INTERRUPT() HAL_timer_disable_interrupt(MF_TIMER_TEMP)

#ifndef HAL_STEP_TIMER_ISR
  #define HAL_STEP_TIMER_ISR() _HAL_TIMER_ISR(MF_TIMER_STEP)
#endif
#ifndef HAL_TEMP_TIMER_ISR
  #define HAL_TEMP_TIMER_ISR() _HAL_TIMER_ISR(MF_TIMER_TEMP)
#endif

void HAL_timer_0_callback();
void HAL_timer_1_callback();
void HAL_timer_2_callback();
void HAL_timer_3_callback();

extern volatile hal_timer_t HAL_timer_base[NUM_ALARMS],     // Counter value at the last compare match
                            HAL_timer_compare[NUM_ALARMS];  // Current period, in ticks since the base

// ------------------------
// Public functions
// ------------------------

void HAL_timer_init();
void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);
void HAL_timer_stop(const uint8_t timer_num);

FORCE_INLINE static hal_timer_t HAL_timer_get_count(const uint8_t timer_num) {
  return hal_timer_t(HAL_TIMER_HW->timerawl) - HAL_timer_base[timer_num];
}

FORCE_INLINE static hal_timer_t HAL_timer_get_compare(const uint8_t timer_num) {
  return HAL_timer_compare[timer_num];
}

FORCE_INLINE static void HAL_timer_set_compare(const uint8_t timer_num, const hal_timer_t compare) {
  HAL_timer_compare[timer_num] = compare;
  HAL_TIMER_HW->alarm[timer_num] = HAL_timer_base[timer_num] + compare; // Also arms the alarm

  // An alarm only fires on an exact match, so one set in the past would wait
  // for the counter to wrap. Raise the interrupt directly in that case.
  if (compare != HAL_TIMER_TYPE_MAX && HAL_timer_get_count(timer_num) >= compare)
    hw_set_bits(&HAL_TIMER_HW->intf, _BV(timer_num));
}

FORCE_INLINE static void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  irq_set_enabled(HAL_TIMER_IRQ(timer_num), true);
}
FORCE_INLINE static void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  irq_set_enabled(HAL_TIMER_IRQ(timer_num), false);
}
FORCE_INLINE static bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  return irq_is_enabled(HAL_TIMER_IRQ(timer_num));
}

inline void HAL_timer_isr_prologue(const uint8_t) {}
inline void HAL_timer_isr_epilogue(const uint8_t) {}
