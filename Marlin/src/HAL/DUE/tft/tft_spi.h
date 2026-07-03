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

#include <stdint.h>

#ifndef LCD_READ_ID
  #define LCD_READ_ID 0x04   // Read display identification information (0xD3 on ILI9341)
#endif
#ifndef LCD_READ_ID4
  #define LCD_READ_ID4 0xD3   // Read display identification information (0xD3 on ILI9341)
#endif

#define DATASIZE_8BIT  SPI_CSR_BITS_8_BIT
#define DATASIZE_16BIT SPI_CSR_BITS_16_BIT
#define DATASIZE_32BIT SPI_CSR_BITS_16_BIT
#define TFT_IO_DRIVER  TFT_SPI
#define DMA_MAX_WORDS  0xFFFF
#define DMA_MINC_ENABLE  1
#define DMA_MINC_DISABLE 0

#ifndef TFT_SPI_CLOCK_DIVIDER
  #define TFT_SPI_CLOCK_DIVIDER 8   // 84 MHz / 8 = 10.5 MHz
#endif

#define TFT_SPI_CHAN 3

class TFT_SPI {
private:
  static uint16_t dataSize;

  static uint32_t readID(const uint16_t inReg);
  static void transmit(uint16_t data);
  static void transmit(uint32_t memoryIncrease, uint16_t *data, uint16_t count);
  static void transmitDMA(uint32_t memoryIncrease, uint16_t *data, uint16_t count);

  static uint8_t transfer8(uint8_t data);
  static uint16_t transfer16(uint16_t data);

public:
  static void init();
  static uint32_t getID();
  static bool isBusy();
  static void abort();

  static void dataTransferBegin(uint16_t dataWidth=DATASIZE_16BIT);
  static void dataTransferEnd() { WRITE(TFT_CS_PIN, HIGH); SPI0->SPI_CR = SPI_CR_SPIDIS; };
  static void dataTransferAbort();

  static void writeData(uint16_t data) { transmit(data); }
  static void writeReg(const uint16_t inReg) { WRITE(TFT_A0_PIN, LOW); transmit(inReg); WRITE(TFT_A0_PIN, HIGH); }

  static void writeSequence_DMA(uint16_t *data, uint16_t count) { transmitDMA(DMA_MINC_ENABLE, data, count); }
  static void writeMultiple_DMA(uint16_t color, uint16_t count) { static uint16_t data; data = color; transmitDMA(DMA_MINC_DISABLE, &data, count); }

  static void writeSequence(uint16_t *data, uint16_t count) { transmit(DMA_MINC_ENABLE, data, count); }
  static void writeMultiple(uint16_t color, uint32_t count) {
    while (count > 0) {
      transmit(DMA_MINC_DISABLE, &color, count > DMA_MAX_WORDS ? DMA_MAX_WORDS : count);
      count = count > DMA_MAX_WORDS ? count - DMA_MAX_WORDS : 0;
    }
  }
};
