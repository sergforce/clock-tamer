/*
        ClockTamer - a software for configurable reference clock
                  Copyright (C) 2015, Fairwaves Inc.
          by Sergey Kostanbaev <Sergey.Kostanbaev@fairwaves.co>
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

#ifndef _TAMER_BOARD_H_
#define _TAMER_BOARD_H_

#include <avr/io.h>

#define JTAG_TMS          PC2
#define JTAG_TCK          PC4
#define JTAG_TDO          PC6 //Actually its TDI on schematic
#define JTAG_TDI          PC5 //Actually its TDO on schematic
#define JTAG_DDR          DDRC
#define JTAG_PORT         PORTC
#define JTAG_PIN          PINC


#define INFOLED           PB5
#define INFOLED_DDR       DDRB
#define INFOLED_PORT      PORTB


#define SPI_SS            PB0
#define SPI_SCK           PB1
#define SPI_MOSI          PB2
#define SPI_MISO          PB3
#define SPI_DDR           DDRB
#define SPI_PORT          PORTB


#define DAC_SYNC          PD4
#define DAC_SYNC_DDR      DDRD
#define DAC_SYNC_PORT     PORTD


#define SYNT_SEL          PB6
#define SYNT_SEL_DDR      DDRB
#define SYNT_SEL_PORT     PORTB


#define ADF_CE            PD5
#define ADF_LD            PD6
#define ADF_DDR           DDRD
#define ADF_PORT          PORTD
#define ADF_PIN           PIND


// TODO:   FO_CPLD=PB4, TP1=PB7, TP2=PD0, SPI_CPLD=PD1, TXD=PD2, RXD=PD3, BOOT=PD7
void BoardInit(void);


#define DacSyncInit()      DAC_SYNC_DDR |= (1 << DAC_SYNC)
#define DacSyncSet()       DAC_SYNC_PORT |= (1 << DAC_SYNC)
#define DacSyncClear()     DAC_SYNC_PORT &= ~(1 << DAC_SYNC)

#define LedInit()          INFOLED_DDR |=  (1 << INFOLED)
#define LedSet()           INFOLED_PORT |= (1 << INFOLED)
#define LedClear()         INFOLED_PORT &= ~(1 << INFOLED)

#define SyntSelInit()      SYNT_SEL_DDR |= (1 << SYNT_SEL)
#define SyntSelSet()       SYNT_SEL_PORT |= (1 << SYNT_SEL)
#define SyntSelClear()     SYNT_SEL_PORT &= ~(1 << SYNT_SEL)

#define AdfInit()          ADF_DDR |= (1 << ADF_CE)
#define AdfCeSet()         ADF_PORT |= (1 << ADF_CE)
#define AdfCeClear()       ADF_PORT &= ~(1 << ADF_CE)


void write_reg_ADF4355(uint8_t f1, uint8_t f2, uint8_t f3, uint8_t f4);
void write_reg_DAC12(uint8_t f1, uint8_t f2);

#define ADF4355_WRITE(x)  write_reg_ADF4355( \
  (uint8_t)((x) >> 24), \
  (uint8_t)((x) >> 16), \
  (uint8_t)((x) >> 8), \
  (uint8_t)((x)))

#define DAC12_WRITE(x)  write_reg_DAC12( \
  (uint8_t)((x) >> 8), \
  (uint8_t)((x)))


void JTAGInit(void);
void JTAGReset(void);
uint32_t JTAGSDR(uint32_t in, uint8_t num_bits);
uint32_t JTAGSIR(uint32_t in, uint8_t num_bits);
void JTAGRunTest(uint32_t clks);



#endif //_TAMER_BOARD_H_
