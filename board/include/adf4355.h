/*
        ClockTamer - a software for configurable reference clock
                  Copyright (C) 2015, Fairwaves Inc.
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

#ifndef ADF4355
#define ADF4355


/* REGISTERS */

/* REG0: {AUTOCAL, PRESCALER, 16-bit INT VALUE}  */
#define REG0_NVALUE_SHIFT       4
#define REG0_NVALUE_MSK         0xffff
#define REG0_PRESCALER_SHIFT    20
#define REG0_AUTOCAL_SHIFT      21

/* REG1: {MAIN FRAC1} */
#define REG1_MFRAC_SHIFT        4
#define REG1_MFRAC_MSK          0xffffff


/* REG2: {AUX FRAC2, AUX MOD2} */
#define REG2_AUX_FRAC_SHIFT     18
#define REG2_AUX_FRAC_MSK       ((1<<14)-1)
#define REG2_AUX_MOD_SHIFT      4
#define REG2_AUX_MOD_MSK        ((1<<14)-1)

/* REG3: {SD LOAD RESET, PHASE RESYNC, PHASE ADJUST, 24bit PHASE} */





#define REG2_AUX_FRAC_SHIFT     18






#endif
