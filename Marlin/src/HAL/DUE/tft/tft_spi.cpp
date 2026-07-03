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

#include "../../platforms.h"

#ifdef __SAM3X8E__

#include "../../../inc/MarlinConfig.h"

#if HAS_SPI_TFT

#include "tft_spi.h"

#define SPI_TFT_SR_READY() (SPI0->SPI_SR & SPI_SR_RDRF)
#define SPI_TFT_SR_WRITE() (SPI0->SPI_SR & SPI_SR_TDRE)

uint16_t TFT_SPI::dataSize = DATASIZE_16BIT;

static inline void tft_spi_configure() {
  pmc_enable_periph_clk(ID_SPI0);
  SPI0->SPI_CR = SPI_CR_SPIDIS;
  SPI0->SPI_CR = SPI_CR_SWRST;
  SPI0->SPI_CR = SPI_CR_SWRST;
  SPI0->SPI_MR = SPI_MR_MSTR | SPI_MR_PS | SPI_MR_MODFDIS;
  SPI0->SPI_CSR[TFT_SPI_CHAN] = SPI_CSR_NCPHA | SPI_CSR_CSAAT | SPI_CSR_DLYBCT(1) | SPI_CSR_SCBR(TFT_SPI_CLOCK_DIVIDER) | DATASIZE_16BIT;
}

void TFT_SPI::init() {
  WRITE(TFT_A0_PIN, HIGH);
  WRITE(TFT_CS_PIN, HIGH);

  PIO_Configure(g_APinDescription[PIN_SPI_MISO].pPort, g_APinDescription[PIN_SPI_MISO].ulPinType, g_APinDescription[PIN_SPI_MISO].ulPin, g_APinDescription[PIN_SPI_MISO].ulPinConfiguration);
  PIO_Configure(g_APinDescription[PIN_SPI_MOSI].pPort, g_APinDescription[PIN_SPI_MOSI].ulPinType, g_APinDescription[PIN_SPI_MOSI].ulPin, g_APinDescription[PIN_SPI_MOSI].ulPinConfiguration);
  PIO_Configure(g_APinDescription[PIN_SPI_SCK].pPort,  g_APinDescription[PIN_SPI_SCK].ulPinType,  g_APinDescription[PIN_SPI_SCK].ulPin,  g_APinDescription[PIN_SPI_SCK].ulPinConfiguration);

  tft_spi_configure();
}

void TFT_SPI::dataTransferBegin(uint16_t dataWidth) {
  dataSize = dataWidth == DATASIZE_8BIT ? DATASIZE_8BIT : DATASIZE_16BIT;
  SPI0->SPI_CR = SPI_CR_SPIEN;
  SPI0->SPI_CSR[TFT_SPI_CHAN] = SPI_CSR_NCPHA | SPI_CSR_CSAAT | SPI_CSR_DLYBCT(1) | SPI_CSR_SCBR(TFT_SPI_CLOCK_DIVIDER) | dataSize;
   WRITE(TFT_CS_PIN, LOW);
}

#include "../../../lcd/tft_io/tft_ids.h"

uint32_t TFT_SPI::getID() {
  uint32_t id = readID(LCD_READ_ID);

  if ((id & 0xFFFF) == 0 || (id & 0xFFFF) == 0xFFFF) {
    id = readID(LCD_READ_ID4);
  }

  #ifdef TFT_DEFAULT_DRIVER
    if ((id & 0xFFFF) == 0 || (id & 0xFFFF) == 0xFFFF) id = TFT_DEFAULT_DRIVER;
  #endif

  return id;
}

uint32_t TFT_SPI::readID(const uint16_t inReg) {
  uint32_t data = 0;

  #if PIN_EXISTS(TFT_MISO)
    dataTransferBegin(DATASIZE_8BIT);
    writeReg(inReg);

    for (uint8_t i = 0; i < 4; i++) {
      data <<= 8;
      data |= transfer8(0x00);
    }

    dataTransferEnd();
  #endif

  return data >> 7;
}

bool TFT_SPI::isBusy() {
  return false;
}

void TFT_SPI::abort() {
  dataTransferEnd();
}

void TFT_SPI::dataTransferAbort() {
  abort();
}

uint8_t TFT_SPI::transfer8(const uint8_t data) {
  while (!SPI_TFT_SR_WRITE()) {}
  SPI0->SPI_TDR = uint32_t(data) | SPI_PCS(TFT_SPI_CHAN);
  while (!SPI_TFT_SR_READY()) {}
  return uint8_t(SPI0->SPI_RDR);
}

uint16_t TFT_SPI::transfer16(const uint16_t data) {
  while (!SPI_TFT_SR_WRITE()) {}
  SPI0->SPI_TDR = uint32_t(data) | SPI_PCS(TFT_SPI_CHAN);
  while (!SPI_TFT_SR_READY()) {}
  return uint16_t(SPI0->SPI_RDR);
}

void TFT_SPI::transmit(uint16_t data) {
  if (dataSize == DATASIZE_8BIT) transfer8(uint8_t(data));
  else transfer16(data);
}

void TFT_SPI::transmit(uint32_t memoryIncrease, uint16_t *data, uint16_t count) {
  while (count--) {
    transmit(*data);
    if (memoryIncrease == DMA_MINC_ENABLE) ++data;
  }
}

void TFT_SPI::transmitDMA(uint32_t memoryIncrease, uint16_t *data, uint16_t count) {
  dataTransferBegin(DATASIZE_16BIT);
  transmit(memoryIncrease, data, count);
  TERN_(TFT_SHARED_IO, while (isBusy()));
}

#endif // HAS_SPI_TFT
#endif // __SAM3X8E__
