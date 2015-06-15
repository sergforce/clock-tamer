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

/** \file
 *
 *  Header file for Tamer.c.
 */

#ifndef _USB_SERIAL_H_
#define _USB_SERIAL_H_

/* Includes: */
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>

#include "DescriptorsCDC.h"
#include "DescriptorsDFU.h"
#include "BootloaderDFU.h"

#if TAMER_VER < 200
#include "TamerBoard.h"
#else
#include "TamerBoard2.h"

#define USE_HEX_OUTPUT
#endif

#include "RingBuff.h"

#include "TamerProtocol.h"

#include <LUFA/Version.h>
#include <LUFA/Drivers/USB/USB.h>


uint8_t OnCmdGPS(void);

uint8_t ProcessCommand(void);

void SetupHardware(void);

/**
 * @brief Store Store byte to send to the host
 * @param byte byte to store
 */
void Store(uint8_t byte);

#endif
