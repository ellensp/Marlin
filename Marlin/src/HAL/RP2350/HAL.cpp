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

#include "HAL.h"
//#include "usb_serial.h"

#include "../../inc/MarlinConfig.h"
#include "../shared/Delay.h"

extern "C" {
  #include "pico/bootrom.h"
  #include "hardware/watchdog.h"
  #include "hardware/adc.h"
  #include "pico/time.h"
}

#if HAL_SD_HOST_DRIVE
  #include "msc_sd.h"
#endif

// ------------------------
// Public Variables
// ------------------------

// ADC layout from the Arduino-Pico core, which knows the chip variant. (The SDK's NUM_ADC_CHANNELS and
// ADC_FIRST_PIN are for the RP2350B here, because the core sets PICO_RP2350A after the SDK header is read.)
//   RP2350A: GPIO 26-29 are channels 0-3, MCU temperature is channel 4
//   RP2350B: GPIO 40-47 are channels 0-7, MCU temperature is channel 8
#define ADC_FIRST_PIN     __FIRSTANALOGGPIO
#define ADC_TEMP_CHANNEL  (__GPIOCNT - __FIRSTANALOGGPIO)

// ------------------------
// Public functions
// ------------------------

TERN_(POSTMORTEM_DEBUGGING, extern void install_min_serial());

// HAL initialization task
void MarlinHAL::init() {
  // Ensure F_CPU is a constant expression.
  // If the compiler breaks here, it means that delay code that should compute at compile time will not work.
  // So better safe than sorry here.
  constexpr int cpuFreq = F_CPU;
  UNUSED(cpuFreq);

  #if HAS_MEDIA && DISABLED(ONBOARD_SDIO) && PIN_EXISTS(SD_SS)
    OUT_WRITE(SD_SS_PIN, HIGH); // Try to set SD_SS_PIN inactive before any other SPI users start up
  #endif

  #if PIN_EXISTS(LED)
    OUT_WRITE(LED_PIN, LOW);
  #endif

  #if ENABLED(SRAM_EEPROM_EMULATION)
    // __HAL_RCC_PWR_CLK_ENABLE();
    // HAL_PWR_EnableBkUpAccess();           // Enable access to backup SRAM
    // __HAL_RCC_BKPSRAM_CLK_ENABLE();
    // LL_PWR_EnableBkUpRegulator();         // Enable backup regulator
    // while (!LL_PWR_IsActiveFlag_BRR());   // Wait until backup regulator is initialized
  #endif

  HAL_timer_init();

  #if ALL(EMERGENCY_PARSER, USBD_USE_CDC)
    USB_Hook_init();
  #endif

  TERN_(POSTMORTEM_DEBUGGING, install_min_serial());    // Install the min serial handler

  TERN_(HAL_SD_HOST_DRIVE, MSC_SD_init());              // Enable USB SD card access

  #if PIN_EXISTS(USB_CONNECT)
    OUT_WRITE(USB_CONNECT_PIN, !USB_CONNECT_INVERTING); // USB clear connection
    delay_ms(1000);                                     // Give OS time to notice
    WRITE(USB_CONNECT_PIN, USB_CONNECT_INVERTING);
  #endif
}

void MarlinHAL::idletask() { TERN_(HAL_SD_HOST_DRIVE, tuh_task()); }

uint8_t MarlinHAL::get_reset_source() {
  switch (rp2040.getResetReason()) {
    case RP2040::PWRON_RESET:     return RST_POWER_ON;
    case RP2040::RUN_PIN_RESET:   return RST_EXTERNAL;
    case RP2040::SOFT_RESET:      return RST_SOFTWARE;
    case RP2040::WDT_RESET:       return RST_WATCHDOG;
    case RP2040::DEBUG_RESET:     return RST_JTAG;
    case RP2040::GLITCH_RESET:
    case RP2040::BROWNOUT_RESET:  return RST_BROWN_OUT;
    default:                      return 0;
  }
}

void MarlinHAL::reboot() { watchdog_reboot(0, 0, 1); }

// ------------------------
// Watchdog Timer
// ------------------------

#if ENABLED(USE_WATCHDOG)

  #define WDT_TIMEOUT_US TERN(WATCHDOG_DURATION_8S, 8000000, 4000000) // 4 or 8 second timeout

  extern "C" {
    #include "hardware/watchdog.h"
  }

  void MarlinHAL::watchdog_init() {
    #if DISABLED(DISABLE_WATCHDOG_INIT)
      static_assert(WDT_TIMEOUT_US > 1000, "WDT Timeout is too small, aborting");
      watchdog_enable(WDT_TIMEOUT_US / 1000, true);
    #endif
  }

  void MarlinHAL::watchdog_refresh() {
    watchdog_update();
    #if DISABLED(PINS_DEBUGGING) && PIN_EXISTS(LED)
      TOGGLE(LED_PIN); // Heartbeat indicator
    #endif
  }

#endif // USE_WATCHDOG

// ------------------------
// ADC
// ------------------------

/**
 * Marlin's Temperature ISR calls adc_start on one tick and adc_ready / adc_value on the next,
 * 1ms later. adc_start only switches the input, so a high-impedance source has that 1ms to
 * settle. adc_value then does the conversion, which takes 2us. There is no need for
 * free-running mode, the FIFO, or a second core.
 */

static inline uint8_t adc_channel(const pin_t pin) {
  return pin == HAL_ADC_MCU_TEMP_DUMMY_PIN ? ADC_TEMP_CHANNEL : pin - ADC_FIRST_PIN;
}

void MarlinHAL::adc_init() {
  analogReadResolution(HAL_ADC_RESOLUTION);
  ::adc_init();
}

void MarlinHAL::adc_enable(const pin_t pin) {
  if (pin == HAL_ADC_MCU_TEMP_DUMMY_PIN)
    adc_set_temp_sensor_enabled(true);
  else if (WITHIN(pin, ADC_FIRST_PIN, ADC_FIRST_PIN + ADC_TEMP_CHANNEL - 1))
    adc_gpio_init(pin);
}

static uint8_t adc_selected;

void MarlinHAL::adc_start(const pin_t pin) {
  adc_selected = adc_channel(pin);
  adc_select_input(adc_selected);
}

uint16_t MarlinHAL::adc_value() {
  // An analogRead() from the main loop (e.g., M43) may have switched the input since adc_start
  if (adc_get_selected_input() != adc_selected) adc_select_input(adc_selected);
  return adc_read();
}

// Reboot into the UF2 bootloader to initiate a firmware flash
void flashFirmware(const int16_t) { reset_usb_boot(0, 0); }

extern "C" {
  void * _sbrk(int incr);
  extern char __StackLimit;    // Lowest address the stack can grow to
}

// Return free memory between end of heap and start of stack
int freeMemory() {
  void* heap_end = _sbrk(0);
  // Use the linker-provided stack limit instead of a local variable
  // __StackLimit is the lowest address the stack can grow to
  return (char*)&__StackLimit - (char*)heap_end;
}

#endif // __PLAT_RP2350__
