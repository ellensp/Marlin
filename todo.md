# RP2350 HAL: to do

Work list for the RP2350 port (branch `rp2350`, HAL in `Marlin/src/HAL/RP2350/`, board `BOARD_RASPBERRY_PI_PICO2`, env `RP2350`). Started 2026-11-27. This file is a temporary progress note and will be removed when the branch is cleaned up for a pull request.

Related: the RP2040 HAL has the same kind of stepper timer limit (10 µs resolution, 100 kHz); not addressed here.

**Build note:** after changing a HAL header, run `pio run -e RP2350 -t clean` first. PlatformIO doesn't track headers included through `HAL_PATH()`, so an ordinary build keeps the old objects.

**How things were tested:** on a bare Pico 2 over USB, nothing wired to it. "Bench" below means that. A temporary test configuration was used for some checks (EEPROM_SETTINGS, PINS_DEBUGGING, MARLIN_DEV_MODE, SHOW_TEMP_ADC_VALUES, TEMP_SENSOR_SOC 100, TEMP_SENSOR_BOARD 1, FAST_PWM_FAN); `Configuration.h` and `Configuration_adv.h` were put back afterwards. Later tests used an ESP32 test rig wired to the Pico 2 (see "Hardware testing outstanding"); those are marked "rig". The test scripts and rig firmware are not part of this branch.

## Code: done (2026-11-27)

- [x] **Stepper and temperature timers** (`timers.h`, `timers.cpp`). TIMER1 alarms at 2 MHz (12 MHz since the rig tests, see below) with their own interrupts, replacing the SDK alarm pools and the 100 kHz rate copied from the RP2040. Stepper interrupt is top priority, temperature lowest.
  - Bench: single X moves. Old code ran up to 93% long above about 100 kHz step rate and dropped USB at 640 kHz. New code is on time through 320 kHz. `G4 P2000` took 2.001 s.
  - **Needs:** everything under "New timer code" below.
- [x] **1. Fast pin access** (`fastio.h`). `WRITE` / `READ` / `TOGGLE` now use `gpio_put` / `gpio_get` (one register write) where they called `digitalWrite` / `digitalRead`. `IS_INPUT`, `IS_OUTPUT` and `_GET_MODE` now return the pin direction. `PWM_PIN` stays true: every GPIO has a PWM output.
  - Bench: the 640 kHz move went from 90% long to 33% long, so the stepper ISR is quicker. Endstop reads (`M119`) still work.
  - **Needs:** a scope or step counter on real STEP/DIR pins. `WRITE` no longer takes a pin back from PWM or the ADC, or changes pulls, so check any feature that uses both `WRITE` and PWM on one pin, or writes to an input pin.
- [x] **2. PWM** (`fast_pwm.cpp`). Rewritten on the SDK's PWM slices, without the core's `analogWrite`. `set_pwm_duty` now honours `v_size` and `invert`. `set_pwm_frequency` sets one slice (the pins that share it, e.g. GP14 and GP15) and keeps the duty of both outputs.
  - Bench: fan on GP28 with `FAST_PWM_FAN_FREQUENCY 2573`, duty read back through the ADC on the same pin. `M106 S0/26/64/128/191/255` measured 0.2 / 11.1 / 24.1 / 50.6 / 75.4 / 99.9% (expected 0 / 10.2 / 25.1 / 50.2 / 74.9 / 100%).
  - **Needs:** the frequency measured (only duty was checked), `invert` and `v_size` other than 255, two pins on one slice, a real fan, and laser/spindle PWM. `TFT_BACKLIGHT_PIN` code calls `analogWrite` directly and would mix the two schemes.
- [x] **3. ADC without a dedicated core** (`HAL.cpp`, `HAL.h`). Core 1 is no longer started. `adc_start` selects the input, and `adc_value` converts it one temperature tick (1 ms) later, which takes 2 µs. The HAL's own 16-sample averaging is gone; Marlin's `OVERSAMPLENR` still applies.
  - As a result: the watchdog is a plain `watchdog_update()` (no cross-core heartbeat), flash EEPROM no longer parks core 1, and `D100 C1` (freeze core 1) is RP2040-only again in `gcode_d.cpp`.
  - Bench: MCU temperature reads 20-21 °C (raw 890). GP28 driven by its own PWM output reads raw 9 at 0% and 4089 at 100%. Watchdog reset on a `D100` hang came 4.7 s after the command (1 s delay + 4 s timeout).
  - **Needs:** real thermistors against a known temperature, noise compared with the old averaged readings, and several sensors at once (the input is switched 1 ms before each conversion; check for crosstalk).
  - **Open question:** GP29 (VSYS/3 on the Pico 2) read raw 1220, about 0.98 V, where about 1.6 V is expected. The old and new ADC code agree, so it isn't settling time. Check GP29 with a meter.
- [x] **4. Reset reason** (`HAL.cpp`, `get_reset_source`). Maps the core's `rp2040.getResetReason()` to power-on, external (RUN pin), software, watchdog, debugger and brown-out (which also covers the supply glitch detector).
  - **Not verified.** Marlin prints the reset reason before USB serial is connected, so it couldn't be read on the bench. The core's own comment says its RP2350 path is untested. Needs a UART console, then: power cycle, RUN pin, `D0`, watchdog.
- [x] **5. Interrupt priorities.** Reviewed, no change. Stepper is 0x00, temperature 0xC0, and everything else (USB, UARTs, endstop pin interrupts) stays at the default 0x80 in between, which is the same ordering as the STM32 HAL. Override with `STEP_TIMER_IRQ_PRIO` / `TEMP_TIMER_IRQ_PRIO`.
- [x] **7. Tidy-ups.** Removed the wrong 12 MHz `F_CPU` fallback from `HAL.h`; corrected the software SPI and flash EEPROM comments.
- [x] **Open-drain outputs** (`fastio.h`, `fastio.cpp`). They couldn't work: the core's `pinMode` has no open-drain mode. Now emulated: `OUT_WRITE_OD` / `SET_OUTPUT_OD` hold the output latch low and `WRITE` / `TOGGLE` switch the pin direction for pins in the `fastio_od_pins` mask.
  - Bench: compiles; the 640 kHz move time is unchanged by the extra mask test in `WRITE`. **Not run**: nothing in the test builds uses an open-drain pin. Needs a pin with a pull-up and a meter or the ESP32.
- [x] **`OUT_WRITE` no longer glitches low** (`fastio.cpp`, `fastio_out_write`). The pin is made a floating input, the latch is set, then the output is enabled.
  - Bench: read back with `M43`: stepper enable (GP7) is high after boot, low after `M17`, high after `M18`; a soft-PWM fan on GP28 follows `M106 S255` / `S0`. **Needs** a scope to confirm there is no low pulse at boot.
- [x] **`M43` analog reads** (`pinsDebug.h`). `analog_index` now returns the GPIO number, which is what the core's `analogRead` takes (it returned 0 for every pin before). The `(A n)` label still shows the channel.
  - Bench: `M43 P29 I` now reports a reading. It was 439-443 against the temperature ISR's 1220 for the same pin; probably because `analogRead` converts straight after switching the input and the VSYS divider is high impedance (not confirmed). Debug output only.
- [x] **ADC input guard** (`HAL.cpp`, `adc_value`). An `analogRead` from the main loop (e.g. `M43`) switches the ADC input, which could have made the temperature ISR convert the wrong channel. `adc_value` now re-selects its input if it was changed.
- [x] **CI** (`buildroot/tests/RP2350/config-01.ini`, `config-02.ini`, `.github/workflows/ci-build-tests.yml`). Config 1 is the default build for `BOARD_RASPBERRY_PI_PICO2`; config 2 adds flash EEPROM, MCU temperature, `FAST_PWM_FAN`, endstop interrupts, `PINS_DEBUGGING`, `DIRECT_PIN_CONTROL` and `MARLIN_DEV_MODE`. Both build from the stock configuration files with `mftest -t RP2350`. No servo test: the Pico 2 pins file has no `SERVO0_PIN`.

## Code: still to do

- [ ] **6. Limits copied into `inc/SanityCheck.h`.** Not attempted; both are new features, not rewrites.
  - TFT UIs (`TFT_COLOR_UI`, `TFT_LVGL_UI`, `TFT_CLASSIC_UI`) are blocked. With 520 KB of RAM they may be possible, but the HAL has no `tft/` code.
  - Serial ports are limited to USB and two UARTs. PIO UARTs (`SerialPIO` in the core) could add more.
- [x] **Flash EEPROM write waits for motion** (`eeprom_flash.cpp`). The rig measured a 24 ms gap in the step train when `M500` wrote flash during a move. The write now calls `planner.synchronize()` first: `M500` sent during a move replies when the move ends (2.5 s in the test), with no gap and exact step counts; idle it still takes 27 ms. `PRINTCOUNTER` stays blocked with flash EEPROM, since it would pause a print at each save.

## Hardware testing outstanding

The test rig is an ESP32 dev board with an ADS1115 as a test controller: step counting and pulse capture, simulated endstops, DAC-driven thermistor inputs read back by the ADS1115, and PWM measurement. Built 2026-11-27; it takes line commands over its USB serial port from scripts on the PC. Pulse capture holds 512 pulses, started from the PC part-way through a move; a second capture of a continuous fast signal returns nothing (rig fault, use `PWM` there). The ADS1115 was rewired later the same day: A0 and A1 on the DAC lines, A2 on the Pico's ADC_VREF (pin 35), A3 on the ESP32's 3.3 V rail. With real thermistor types, the DACs have to be set before the Pico boots; with nothing on the inputs it halted at power-up (see below), though with the ADS1115 attached a boot with the DACs off did not halt. Wiring (both boards are 3.3 V; share ground, leave the 3.3 V rails unconnected, no pull-ups on GP26 / GP27):

| Pico 2 | Signal | ESP32 | Used for |
| --- | --- | --- | --- |
| GP3 | X step | GPIO34 | step count, pulse width |
| GP6 | X dir | GPIO35 | count direction |
| GP8 | Y step | GPIO36 (VP) | step count |
| GP9 | Y dir | GPIO39 (VN) | count direction |
| GP12 | E step | GPIO27 | step count |
| GP13 | E dir | GPIO14 | count direction |
| GP7 | enable | GPIO13 | driver enable state |
| GP14 | hotend heater | GPIO18 | PWM duty and frequency |
| GP15 | bed heater | GPIO19 | PWM duty and frequency |
| GP28 | fan | GPIO21 | PWM duty and frequency |
| GP0 | X endstop | GPIO22 | simulated endstop |
| GP1 | Y endstop | GPIO23 | simulated endstop |
| GP2 | Z endstop | GPIO4 | simulated endstop |
| GP26 | hotend thermistor | GPIO25 (DAC1) and ADS1115 A0 | simulated temperature |
| GP27 | bed thermistor | GPIO26 (DAC2) and ADS1115 A1 | simulated temperature |
| RUN (pin 30) | reset | GPIO5 | lets the rig restart the Pico |
| GP16 / GP17 | UART0 TX / RX | FT232 adapter on the PC | serial console |
| GP4 / GP5 | UART1 TX / RX | GPIO16 / GPIO17 | second serial port |
| GND | ground | GND | common ground |

ADS1115 stays on GPIO32 / 33. GPIO16 / 17 are free for testing the Pico's UARTs later.

### New timer code

- [x] Step pulse widths on X STEP (rig, 2026-11-27): high 0.88 to 1.29 µs at every rate; low never under 0.86 µs.
- [x] Step interval at cruise (rig, 2026-11-27, 510 periods each): 16 kHz mean 62498 ns (ideal 62500), spread 61975 to 63112; 100 kHz mean 10000 ns, spread 9412 to 10575; 320 kHz mean 3002 ns (ideal 3125), spread 1825 to 5462.
- [x] Step counts (rig, 2026-11-27): 32000, 200000, 640000 and 1280000 steps commanded at 16, 100, 320 and 640 kHz; all counted exactly, and net 0 after the return move.
- [x] Temperature interrupt rate at idle: 1008 Hz (rig, 2026-11-27, from fan soft PWM at 7.87 Hz x 128).
- [x] **Stepper overload no longer stops the temperature interrupt and USB** (`timers.cpp`, `HAL_timer_step_irq`). Before: the temperature interrupt stopped at 480 kHz and USB dropped at 640 kHz (A4988 timing); with `LIN_ADVANCE` and input shaping, at 240 and 320 kHz. Cause: an overloaded Stepper ISR asks to be called again 1 µs later (Marlin core, all platforms), which here is over before anything else can start. Now the Stepper ISR is not called again until 1/8 of the time it just ran has passed (`HAL_STEP_TIMER_SHARE`). After: temperature interrupt at 1 kHz and USB up at every rate tried (640 kHz A4988, 480 kHz with `LIN_ADVANCE` and input shaping, 1.6 MHz TMC2209 timing), step counts exact; overloaded moves take longer (640 kHz A4988: 4.7 s for a 3.1 s move). `FT_MOTION` speed unchanged. Not compared with another board.
- [x] **Timer runs at 12 MHz** (`timers.h`, `HAL_TIMER_RATE`; one tick per crystal cycle, was 2 MHz). At 2 MHz step pulses could be shorter than `MINIMUM_STEPPER_PULSE_NS` (388 ns for 500, 4775 for 5000) because Marlin times them in timer ticks. At 12 MHz: 475 for 500, 5038 for 5000, 1012 to 1150 for the A4988's 1000. Step counts, `LIN_ADVANCE`, input shaping, babystepping and `FT_MOTION` give the same results as at 2 MHz.
- [x] Maximum step rate with TMC2209 timing (rig, 2026-11-27): on time through 960 kHz, about 1 MHz reached when 1.28 MHz is asked for, counts exact to 4.8 million steps, pulses 225 to 238 ns high.
- [ ] The first step pulse of each Stepper ISR is sometimes much longer than the minimum (4 µs for a 1 µs setting in one build, 1 µs in another). Probably flash cache misses, as the ISR runs from flash; not confirmed. Harmless to drivers. Running the ISR from RAM would remove it and should cut jitter.
- [ ] Real stepper drivers and motors: no lost steps over long moves, in both directions.
- [x] What limits the step rate above 320 kHz: at 640 kHz commanded the mean period is 2100 ns (476 kHz), made of about 1.05 µs high and 1.04 µs low. That matches the A4988 timing in the configuration (1 µs minimum pulse, 500 kHz maximum), so it is the driver setting and not the timer. Not tried with a faster driver type.
- [x] Input shaping (rig, 2026-11-27, 40 Hz on X and Y): net step counts exact after a zig-zag and after returning to 0.
- [x] Babystepping with `BABYSTEP_XY` (rig, 2026-11-27): `M290 X0.5` gave 40 steps, Y 20 steps, and 80 extra steps when applied during a move. Needs the axis homed, as designed.
- [x] `LIN_ADVANCE` (rig, 2026-11-27, K 0.2): X exact. E carries about 375 extra steps after an extruding move that runs straight into a travel move, and gives them back at the next full stop; counts were consistent with that to within about 30 steps. Looks like how linear advance works, not checked against another board.
- [ ] `FT_MOTION` (rig, 2026-11-27): moves and zig-zags work, the temperature interrupt keeps running to 320 kHz. **Its step output does not match its own position:** 720000 steps commanded gave 719998 on the pins and did not return to 0 (+4); 24000 gave 24004 pulses with a net of 23996. `M114` showed the full count each time, and `M493 S0` in the same firmware was exact. Same numbers at 2 and 12 MHz and with 5 µs DIR delays, so probably `FT_MOTION` itself, not this HAL. Ruled out: timer resolution, DIR setup time, and the counter (the standard stepper is exact on the same wiring up to 4.8 million steps). Not tried on another board.
- [ ] A long print, for stability.
- [x] Delay accuracy, by pulse width (rig, 2026-11-27): see the 12 MHz timer entry above. `DELAY_NS` itself is used for DIR delays only and was not measured separately.
- [ ] A board with a crystal that is not 12 MHz. The timer now ticks once per crystal cycle and `HAL_TIMER_RATE` is fixed at 12000000; there is a compile-time check against `XOSC_HZ` only.

### New pin, PWM, ADC and reset code

- [x] Step, direction and enable outputs seen on the pins for X, Y and E (rig, 2026-11-27).
- [x] PWM frequency and shared slice (item 2; rig, 2026-11-27). Fan on GP28: 1000 Hz by default, 25.1% at `S64`, 50.2% at `S128`; with `FAST_PWM_FAN_FREQUENCY 25000` it is 25001 Hz with 12.5 / 25.1 / 50.2% at `S32` / `S64` / `S128` and a 160 ns pulse at `S1`. GP14 and GP15 (both slice 7) driven with `M42`: 1000 Hz, each pin holds its own duty (25.1% with 75.3%, 3.9% with 98.0%, 50.2% with 0), static at 0 and 255.
- [ ] PWM `invert`: nothing in this configuration uses it. Not tested.
- [x] Thermistor inputs GP26 / GP27 (item 3; rig with ADS1115 on the same lines, 2026-11-27). Noise: standard deviation about 3 counts of 4096, the same during a 100 kHz move with the fan PWM running. Crosstalk: at most 6 counts when the other channel moves by 2200. `M43` agrees with the temperature path within a few counts. Accuracy against the ADS1115 with its A2 on the Pico's ADC_VREF (3208 mV while running), 16 points from 0.2 V to 3.05 V: the Pico reads high by about 10 counts of 4096 (8 mV) across most of the range, up to 27 at the bottom on GP26, with gain error under 0.4%. Under 1 °C on a type 1 thermistor. An earlier run that showed +1.4% gain used the ESP32's 3.3 V rail (3262.6 mV) as the reference by mistake. No correction added.
- [ ] GP29 voltage with a meter (item 3, open question).
- [x] Reset reason, read on the UART (item 4; 2026-11-27): `External Reset` for the RUN pin, `Watchdog Reset` after `D100` (5.1 s), `Software Reset` after `D0` and after a reboot from the bootloader. `PowerUp` after unplugging and replugging USB.
- [x] Open-drain outputs (rig, 2026-11-27, `BOARD_OPENDRAIN_MOSFETS`): heater pin reads low when off and high when on with a pull-up on the rig, and stays low with a pull-down, so it is released and not driven; the soft-PWM fan pin likewise. The push-pull build drives high into the pull-down.
- [ ] Low pulse on the enable pin at boot (rig, capture across a reset, 2026-11-27). First RP2350 code: 16.5 µs low, then three more pulses of 1.2 µs (X, Y, Z and E share GP7). With the committed `OUT_WRITE` fix: still four pulses, the first 3.8 µs. Now (`fastio_set_output`): `SET_OUTPUT` leaves an existing output's level alone, giving one 3.1 µs pulse. That last one is Marlin's own `SET_OUTPUT` then `WRITE(HIGH)` order, the same on every platform; removing it needs a core change (`OUT_WRITE(pin, init state)`). Heater pin: no edges.

### Rest of the port

Only the items marked have been tested.

- [x] Boots, USB serial, `M115`, `M105`, `M119` (2026-11-27).
- [x] Flash EEPROM: `M500`, `M501`, settings kept across a software reset and a watchdog reset, `M500` during a move (2026-11-27, with core 1 no longer running).
- [x] Watchdog reset on a main-loop hang (`D100`, 2026-11-27).
- [x] `M43` pin report runs (2026-11-27). It showed `Analog in = 0` for GP29 while the temperature ISR read 1220, so its analog read (`pinsDebug.h`) needs a look.
- [x] Settings survive a power cycle (2026-11-27): `M206 X7.25 Y3.50` saved with `M500`, USB unplugged and replugged, read back the same over the UART. A hard reset through the RUN pin keeps them too.
- [x] Heating against a simulated heater (rig, 2026-11-27; the PC models the heater and feeds the temperature back through the DAC). PID held 150 °C within about 1 °C with the heater pin's soft PWM duty matching the reported power; `M303 E0 S150 C3` completed and printed constants; bed on/off control switches GP15 around its target.
- [x] Thermal protection (rig, 2026-11-27): `Thermal Runaway` after 40.3 s when the temperature was forced 50 °C below target; `Heating failed` after 40.4 s when it never rose; bed `MAXTEMP` within 0.6 s. Each one halted with both heater pins low. Hotend `MAXTEMP` and `MINTEMP` could not be reached: the ESP32 DAC only spans about 0.1 to 3.05 V.
- [ ] **USB stops after any halt.** After `kill()` (seen with `Homing Failed`) the message reaches the host, then the port stops answering and opening it hangs; Marlin turns interrupts off in a halt and USB here needs them. A halt before USB is up (real thermistor types with floating inputs, which read as over-temperature) therefore shows nothing at all and the Pico does not enumerate. Heater pins were low and the reset wire recovers it. Whether to keep USB alive in a halt is undecided; other native-USB boards not compared.
- [ ] Endstops on GP0-GP2 switching, with and without `ENDSTOP_INTERRUPTS_FEATURE`. Done with the rig, 2026-11-27, both ways: each line flips only its own endstop in `M119`; `G28 Y` stops when the endstop is triggered mid-move, backs off 5 mm and finishes at 0; an endstop edge during a normal move does not disturb the step count. A real switch not tried.
- [x] Homing with an endstop that never triggers: stops after 300 mm with `Homing Failed` and halts (rig, 2026-11-27).
- [x] UART0 and UART1 with custom pins (2026-11-27). `SERIAL_PORT 0` on GP16 / GP17 (FT232 adapter), `SERIAL_PORT_2 1` on GP4 / GP5 (ESP32 GPIO16 / 17), USB as `SERIAL_PORT_3`: all three answer and a reply goes only to the port that asked. Baud rates 115200, 250000, 500000 and 1000000 set with `M575`: UART0 50 of 50 good replies and UART1 15 of 15 at each. Streaming 1500 checksummed moves over UART0 at 250000 and at 1000000 baud, with step rates to 480 kHz and the other two ports polled: no bad replies in 4500 lines, about 50 lines/s. Halt messages come out on the UART and, like USB, it stops answering after a halt. Emergency parser and hardware flow control not tested.
- [x] Direction reversals (rig, 2026-11-27): 1000 back-and-forth moves at 400, 1600 and 3200 steps/mm give exactly the expected pulse totals and net 0.
- [ ] TMC drivers over UART (software serial) and SPI.
- [ ] SD card over hardware SPI and software SPI.
- [ ] USB SD host drive (`HAL_SD_HOST_DRIVE`; the env sets `NO_SD_HOST_DRIVE`).
- [ ] I2C: wired EEPROM (`eeprom_wired.cpp`) and an I2C OLED (`u8g_com_rp2350_ssd_i2c.cpp`).
- [ ] An SPI LCD (ST7920 / 12864).
- [ ] Servo and BLTouch.
- [ ] NeoPixel.
- [x] `M997` reboots to the UF2 bootloader (`2e8a:000f`) and a flash from there works (2026-11-27).
- [x] `POSTMORTEM_DEBUGGING` (`HAL_MinSerial.cpp`). It did nothing: no fault vector was connected to Marlin's handler. `install_min_serial()` now installs it with the SDK's `exception_set_exclusive_handler`. `D451 T1` (bad write) and `T4` (bad jump) each print a full report on the UART (registers, PC, backtrace) and the watchdog then restarts the board. `T2` and `T3` do not fault on ARM. Over USB not tried; a report needs a UART. Added to CI `config-02.ini`.
- [ ] An RP2350B board (48 GPIO, 8 ADC channels, PWM slices 8-11). The ADC and PWM code allow for it; nothing has run on one. A USB-only check is possible on a board without header pins: boots with a B board definition, MCU temperature (ADC channel 8 on the B), `M43` listing 48 GPIOs with 40-47 as analog, flash EEPROM. Needs a pins file and a board definition. Analog on GPIO 40-47, PWM on slices 8-11 and outputs above GPIO 31 need wires.
