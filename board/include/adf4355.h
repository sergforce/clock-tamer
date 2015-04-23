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
#define REG0_NVALUE_MASK        ((1<<16)-1)
#define REG0_PRESCALER_SHIFT    20
#define REG0_AUTOCAL_SHIFT      21

/* REG1: {MAIN FRAC1} */
#define REG1_MFRAC_SHIFT        4
#define REG1_MFRAC_MASK         ((1<<24)-1)


/* REG2: {AUX FRAC2, AUX MOD2} */
#define REG2_AUX_FRAC_SHIFT     18
#define REG2_AUX_FRAC_MASK      ((1<<14)-1)
#define REG2_AUX_MOD_SHIFT      4
#define REG2_AUX_MOD_MASK       ((1<<14)-1)
#define REG2_AUX_FRAC_SHIFT     18

/* REG3: {SD LOAD RESET, PHASE RESYNC, PHASE ADJUST, 24bit PHASE} */
#define REG3_SD_LOAD_SHIFT      30
#define REG3_PHASE_RSYNC_SHIFT  29
#define REG3_PHASE_ADJ_SHIFT    28
#define REG3_PHASE_SHIFT        4
#define REG3_PHASE_MASK         ((1<<24)-1)


/* REG4: {MUXOUT 3-bit, REF DBL, REF DIV, 10-bit R, DOUBLE BUFF, CURR 4bit,
 *        REF MODE, MUX LOGIX, PD POL, CP 3-STATE, COUNTER RESET}
 */
#define REG4_MUXOUT_SHIFT       27
#define REG4_MUXOUT_MASK        ((1<<3)-1)
#define REG4_REF_DBL_SHIFT      26
#define REG4_REF_DIV_SHIFT      25
#define REG4_R_SHIFT            15
#define REG4_R_MASK             ((1<<10)-1)
#define REG4_DBL_BUFF_SHIFT     14
#define REG4_CURRENT_SHIFT      10
#define REG4_CURRENT_MASK       ((1<<4)-1)
#define REG4_REF_MODE_SHIFT     9
#define REG4_MUX_LOGIC_SHIFT    8
#define REG4_PD_POL_SHIFT       7
#define REG4_PWR_DOWN_SHIFT     6
#define REG4_CP_3STATE_SHIFT    5
#define REG4_CNTR_RESET         4

/* REG5:  RESERVED */
#define REG5_RESERVED           0x00800025

/* REG6: {GATED BLEED, NEG BLEED, RESERVED 4-bit, FB SEL, CP BLEED CURR, RSVD,
 *        MTLD, RSVD, AUX EN, AUX PWR 2-bit, OUT EN, OUT PWR 2-bit}
 */
#define REG6_GATED_BLEED_SHIFT   30
#define REG6_NEG_BLEED_SHIFT     29
#define REG6_RESERVED_SHIFT      25
#define REG6_RESERVED_VALUE      0xA
#define REG6_FB_SEL_SHIFT        24
#define REG6_RF_DIV_SHIFT        21
#define REG6_RF_DIV_MASK         ((1<<3)-1)
#define REG6_CP_BLEED_CURR_SHIFT 13
#define REG6_CP_BLEED_CURR_MASK  ((1<<8)-1)
#define REG6_MLTD_SHIFT          11
#define REG6_AUX_PWR_EN_SHIFT    9
#define REG6_AUX_PWR_SHIFT       7
#define REG6_AUX_PWR_MASK        ((1<<2)-1)
#define REG6_PWR_EN_SHIFT        6
#define REG6_PWR_SHIFT           4
#define REG6_PWR_MASK            ((1<<2)-1)

/* REG7: {RESERVED 6-bit, LE SYNC, RESERVED 15-bit, LD CYCLE CNT 2-bit,
 *        LOL MODE, FRAC-N PREC 2-bit, LD MODE}
 */
#define REG7_RESERVED_SHIFT      26
#define REG7_RESERVED_VALUE      0x4
#define REG7_LE_SYNC_SHIFT       25
#define REG7_LD_CYCLE_CNT_SHIFT  8
#define REG7_LD_CYCLE_CNT_MASK   ((1<<2)-1)
#define REG7_LOL_MODE_SHIFT      7
#define REG7_FRAC_N_PREC_SHIFT   5
#define REG7_FRAC_N_PREC_MASK    ((1<<2)-1)
#define REG7_LD_MODE_SHIFT       4

/* REG8:  RESERVED */
#define REG8_RESERVED            0x102D0428

/* REG9: {VCO BAND DIV 8-bit, TIMEOUT 10-bit, AUTO LEVEL TO 5-bit,
 *        SYNT LOCK TO 5-bit }
 */
#define REG9_VCO_BAND_SHIFT      24
#define REG9_VCO_BAND_MASK       ((1<<8)-1)
#define REG9_TIMEOUT_SHIFT       14
#define REG9_TIMEOUT_MASK        ((1<<10)-1)
#define REG9_AUTO_LVL_TO_SHIFT   9
#define REG9_AUTO_LVL_TO_MASK    ((1<<5)-1)
#define REG9_SYNT_LOCK_TO_SHIFT  4
#define REG9_SYNT_LOCK_TO_MASK   ((1<<5)-1)

/* REG10: {RESERVED 18-bit, ADC CLOCK DIV 8-bit, ADC CONV, ADC EN} */
#define REG10_RESERVED_SHIFT     14
#define REG10_RESERVED_VALUE     0x300
#define REG10_ADC_CLK_DIV_SHIFT  6
#define REG10_ADC_CLK_DIV_MASK   ((1<<8)-1)
#define REG10_ADC_CONV_SHIFT     5
#define REG10_ADC_EN_SHIFT       4

/* REG11:  RESERVED */
#define REG11_RESERVED           0x0061300B

/* REG12: {RESYNC CLOCK 16-bit, RESERVED 12-bit} */
#define REG12_RESYNC_CLOCK_SHIFT 16
#define REG12_RESYNC_CLOCK_MASK  ((1<<16)-1)
#define REG12_RESERVED_SHIFT     4
#define REG12_RESERVED_VALUE     0x41


#endif
