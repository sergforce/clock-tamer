/*
        ClockTamer - a software for configurable reference clock
                  Copyright (C) 2009,2015 Fairwaves
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

#include <stdint.h>

#include "Tamer.h"
#include "TamerConfig.h"
#include "TamerControl.h"

#include <util/delay.h>
#include <avr/eeprom.h>


static void LoadHwInfo(void)
{
    uint16_t i;
    for (i=0; i<HWI_LEN; i++)
    {
        uint8_t c = eeprom_read_byte((uint8_t*)&eeHWInfo[i]);
        if (c == 0)
            break;
        Store(c);
    }
    FillNewLine();
}


static uint8_t SetDac(uint16_t value)
{
    if (value < 0x1000)
    {
        DAC12_WRITE(value);
        return 1;
    }

    return 0;
}
