/*
        ClockTamer - a software for configurable reference clock
                  Copyright (C) 2009, Fairwaves
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

/* defines for UmCLK */

#include <stdint.h>
#include "Tamer.h"

#define JTAG_PULSE() do { \
        JTAG_PORT |= (1 << JTAG_TCK); \
        _nop();                       \
        JTAG_PORT &= ~(1 << JTAG_TCK); \
        } while(0) 

#define _nop() __asm__ __volatile__ (" nop; \r\n")

void JTAGInit(void)
{
    JTAG_PORT &= ~((1 << JTAG_TMS) |
                   (1 << JTAG_TCK) |
                   (1 << JTAG_TDO)); 
    JTAG_DDR |= (1 << JTAG_TMS) |
                (1 << JTAG_TCK) |
                (1 << JTAG_TDO);
}

// Reset JTAG and enter RunTest
// TMS == 0 after exit
void JTAGReset(void)
{
    JTAG_PORT |= (1 << JTAG_TMS);
    JTAG_PORT &= ~((1 << JTAG_TCK) |
                   (1 << JTAG_TDO));

    for (uint8_t i = 0; i < 8; i++) {
        JTAG_PULSE();
    }

    JTAG_PORT &= ~(1 << JTAG_TMS);
    _nop();
    JTAG_PULSE(); 
}

// in RunTest before
// first bit - MSB 
uint32_t JTAGSDR(uint32_t in, uint8_t num_bits)
{
    JTAG_PORT |= (1 << JTAG_TMS);
    _nop();
    JTAG_PULSE();

    // Select DR Scan state

    JTAG_PORT &= ~(1 << JTAG_TMS);
    _nop();
    JTAG_PULSE();

    // Capture DR State

    _nop();
    JTAG_PULSE();

    // Shift DR
    int32_t out = 0;
    for (; num_bits != 0; --num_bits, in<<=1) {
        if (in & 0x80000000UL)
            JTAG_PORT |= (1 << JTAG_TDO);
        else
            JTAG_PORT &= ~(1 << JTAG_TDO);

        if (num_bits == 1)
            JTAG_PORT |= (1 << JTAG_TMS); // Last cycle, exit SDR

        JTAG_PULSE();

        out >>= 1;
        if ((JTAG_PIN & (1 << JTAG_TDI)) ==  (1 << JTAG_TDI))
            out |= 0x80000000UL;
    }
    
    // Exit DR1

    JTAG_PULSE();
    
    // Update DR

    JTAG_PORT &= ~(1 << JTAG_TMS);
    _nop();
    JTAG_PULSE();

    // TestRun

    return out;
}

uint32_t JTAGSIR(uint32_t in, uint8_t num_bits)
{
    JTAG_PORT |= (1 << JTAG_TMS);
    _nop();
    JTAG_PULSE();

    return JTAGSDR(in, num_bits);
}

// TMS == 0 before and after exit
void JTAGRunTest(uint32_t clks)
{
    for (; clks != 0; --clks) {
        JTAG_PULSE(); 
    }
}



