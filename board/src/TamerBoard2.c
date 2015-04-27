/*
        ClockTamer - a software for configurable reference clock
                  Copyright (C) 2009, Fairwaves
          by Sergey Kostanbaev <Sergey.Kostanbaev@fairwaves.ru>
*/

/*
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Tamer.h"
#include <util/delay.h>

void BoardInit()
{
    LedInit();
    LedClear();

    DacSyncInit();
    SyntSelInit();

    SyntSelSet();

    AdfInit();
    AdfCeSet();

    /* SPI Init, configure only output and sck, mode 0 */
    SPI_DDR |= (1 << SPI_MOSI) | (1 << SPI_SCK) | (1 << SPI_SS);
    SPI_PORT |= (1 << SPI_SS);

    SPCR = (1 << MSTR) | (1 << SPE);

}

static uint8_t spi_write(uint8_t data)
{
    SPCR = (1 << MSTR) | (1 << SPE);

    SPDR = data;
    while (!(SPSR & (1<<SPIF)));
    return SPDR;
}

/*
 * ADF4355 uses only INPUT, latch on rising clock edge
 *
 * SPI MODE SPOL=0 CHPA=0 (SPI Mode 0)
 * 32bit write, MSB First
 * 50ns MAX Cycle = 20Mhz
 */

void write_reg_ADF4355(uint8_t f1, uint8_t f2, uint8_t f3, uint8_t f4)
{
    AdfCeClear();

    spi_write(f1);
    spi_write(f2);
    spi_write(f3);
    spi_write(f4);

    AdfCeSet();
}

/*
 * DAC121S101 uses only INPUT, nsync falls before transaction
 *
 * SPI Mode 0
 * 16bit write, MSB First
 * MAX 30 Mhz
 */
void write_reg_DAC12(uint8_t f1, uint8_t f2)
{
        DacSyncSet();
        DacSyncSet(); // Delay of one cycle
        DacSyncClear();

        spi_write(f1);
        spi_write(f2);
}


