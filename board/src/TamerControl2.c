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

#include <stdint.h>

#include "Tamer.h"
#include "TamerConfig.h"
#include "TamerControl.h"

#include <util/delay.h>
#include <avr/eeprom.h>

#define BIND static
#include "uint64_ops.h"

#include "adf4355.h"

//////////////////////////////////////
#define SELF_TESTING
//////////////////////////////////////

#define BLINK_1PPS

#define VCO_FIXED
//#define NO_VERSION
//#define NO_HWINFO
//#define NO_CMDINFO
//#define NO_CMDPIN
//#define NO_CMDREG
//#define NO_CMDEELOAD

#define DEBUG_REGS

extern TamerCommand_t command;
extern RingBuff_t USARTtoUSB_Buffer;

const uint8_t newLine[] PROGMEM = "\r\n";

void FillResultNoNewLinePM(const uint8_t* res);

void FillResultPM(const uint8_t* res)
{
    FillResultNoNewLinePM(res);
    FillResultNoNewLinePM(newLine);
}

static void Store(uint8_t byte)
{
    // May be completly useless, need  ATOMIC_BLOCK(ATOMIC_FORCEON)
    cli();
    Buffer_StoreElement(&USARTtoUSB_Buffer, byte);
    sei();
}

void FillResultNoNewLinePM(const uint8_t *res)
{
    uint8_t byte;
    while ((byte = pgm_read_byte(res++)))
    {
        Store(byte);
    }
}

static void FillHead(const uint8_t* res, uint8_t idx)
{
    uint8_t byte;
    uint8_t i;

    if (idx > 0xf0)
    {
        Store('?');
    }
    else
    {
        res = res + 3*idx;
        for (i = 0; i < 3; i++)
        {
            byte = pgm_read_byte(res++);
            Store(byte);
        }
    }
}

extern uint8_t pCmd[];
extern uint8_t pTrg[];
extern uint8_t pDet[];

static void FillCmd(void)
{
    if (command.cmd > 0)
        FillHead(pCmd, command.cmd - 1);
    Store(',');
    if (command.type > 0)
        FillHead(pTrg, command.type - 1);
    Store(',');
    if (command.details > 0)
        FillHead(pDet, command.details - 1);
    Store(',');
}

static void FillUint32(uint32_t val)
{
    uint32_t stv = 1000000000;
    uint8_t f = 0;

    for (;stv > 0; stv/=10)
    {
        uint8_t v = val / stv;
        // if ((f) || (v > 0))
        {
            f = 1;
            Store('0' + v);
        }
        val -= v*stv;
    }
    if (f == 0)
        Store('0');
}

static void FillCharHex(uint8_t c)
{
    if (c < 10)
        Store('0' + c);
    else
        Store('A' + c - 10);
}

static void FillUint8Hex(uint8_t val)
{
    FillCharHex(val >> 4);
    FillCharHex(val & 0xf);
}

static void FillUint16Hex(uint16_t val)
{
    union {
        uint16_t v;
        uint8_t d[2];
    } m;

    m.v = val;

    FillUint8Hex(m.d[1]);
    FillUint8Hex(m.d[0]);
}

static void FillUint32Hex(uint32_t val)
{
    union {
        uint32_t v;
        uint8_t d[4];
    } m;

    m.v = val;

    FillUint8Hex(m.d[3]);
    FillUint8Hex(m.d[2]);
    FillUint8Hex(m.d[1]);
    FillUint8Hex(m.d[0]);
}

#define FillUint16(x)   FillUint32(x)

const uint8_t resBadRange[] PROGMEM = "Bad tuning range";
const uint8_t resOk[] PROGMEM = "OK";

#ifndef NO_VERSION
#if TAMER_VER == 200
const uint8_t resVersion[] PROGMEM = "ClockTamer2/UmCLKv1.0 SW=2.0 API=2 REV=0.001";
#elif TAMER_VER == 130
const uint8_t resVersion[] PROGMEM = "ClockTamer SW=1.30 API=1";
#elif TAMER_VER == 123
const uint8_t resVersion[] PROGMEM = "ClockTamer SW=1.23 API=1";
#elif TAMER_VER == 122
const uint8_t resVersion[] PROGMEM = "ClockTamer SW=1.22 API=1";
#elif TAMER_VER == 121
const uint8_t resVersion[] PROGMEM = "ClockTamer SW=1.21 API=1";
#elif TAMER_VER == 12
const uint8_t resVersion[] PROGMEM = "ClockTamer SW=1.2 API=1";
#elif TAMER_VER == 11
const uint8_t resVersion[] PROGMEM = "ClockTamer SW=1.1 API=1";
#elif TAMER_VER == 10
const uint8_t resVersion[] PROGMEM = "ClockTamer SW=1.0 API=1";
#else
const uint8_t resVersion[] PROGMEM = "ClockTamer SW=[unknown] API=1";
#endif
#endif




//#define NOVARS

#ifdef NOVARS

#define  Fosc      DEF_Fosc
#define  Fout      DEF_Fout
#define  VCO_MIN   DEF_VCO_MIN
#define  VCO_MAX   DEF_VCO_MAX
#define  VCO_Kbit  DEF_VCO_Kbit

#else
uint32_t Fosc = DEF_Fosc;
uint32_t Fout = DEF_Fout;

#ifdef VCO_FIXED
#define  VCO_MIN   DEF_VCO_MIN
#define  VCO_MAX   DEF_VCO_MAX
#define  VCO_Kbit  DEF_VCO_Kbit

#else
uint16_t VCO_MIN =  DEF_VCO_MIN;
uint16_t VCO_MAX =  DEF_VCO_MAX;
uint16_t VCO_Kbit = DEF_VCO_Kbit;
#endif
#endif


#if TAMER_VER < 200
uint8_t LMK_OutMask = DEF_OUT_MASK_LMK;
uint8_t LMK_devider;
#endif

uint8_t AutoFreq;
#ifdef PRESENT_DAC12
uint16_t DacValue;
#endif

#ifdef PRESENT_GPS
uint8_t GpsSync_divider = 1;  //DIV
uint8_t PPS_skipped;          //KBT
uint32_t CounterHHValue;      //R00
uint16_t Count1PPS;           //R01
uint32_t LastOCPVal;          //R02
uint32_t FilteredVal;         //R03
uint32_t ddd;                 //MAX
uint8_t AutoUpdateGps;        //AUT
uint32_t LastAutoUpd;         //MIN
#endif

#if TAMER_VER >= 12 && TAMER_VER < 200
uint8_t EnableOscillator = 1;
#endif

#ifdef SELF_TESTING
uint8_t VCO_locked = 0;
#ifdef PRESENT_GPS
uint8_t GPS_locked = 0;
#endif

uint8_t SelfStage = 0;
uint8_t SelfStageMax = 0;

uint32_t SelfMin = 0;
uint32_t SelfMax = 0;
uint32_t SelfPrev = 0;


#define FOLD_VALUE  FOLD_DIGITAL_LOCK

static inline uint8_t IsVcoLocked(void)
{
    return (PINC & (1<<PC5));
}
#else //SELF_TESTING
#define FOLD_VALUE FOLD_DISABLED
#endif

#define GPSSYNC_MAX_FREQ        3000000


#ifdef DEBUG_REGS
uint32_t tmp_r0;
uint32_t tmp_r1;
uint32_t tmp_r2;
uint32_t tmp_r3;
uint32_t tmp_lmk;
#endif

uint32_t eeFosc         EEMEM = DEF_Fosc;
uint32_t eeFout         EEMEM = DEF_Fout;

#if TAMER_VER < 200
# ifndef VCO_FIXED
uint16_t eeVCO_MIN      EEMEM = DEF_VCO_MIN;
uint16_t eeVCO_MAX      EEMEM = DEF_VCO_MAX;
uint16_t eeVCO_Kbit     EEMEM = DEF_VCO_Kbit;
# else
uint16_t unused_eeVCO_MIN      EEMEM = DEF_VCO_MIN;
uint16_t unused_eeVCO_MAX      EEMEM = DEF_VCO_MAX;
uint16_t unused_eeVCO_Kbit     EEMEM = DEF_VCO_Kbit;
# endif
uint8_t  eeLMK_OutMask  EEMEM = DEF_OUT_MASK_LMK;
#else
uint32_t ee_adf4355_regs[13] EEMEM;
#endif

uint8_t  eeAutoFreq     EEMEM = 1;
uint16_t eeDacValue     EEMEM = 2048;

#ifdef GPS_ENABLE
uint8_t  eeAutoGPSSync  EEMEM = 1;
#else
uint8_t  eeAutoGPSSync  EEMEM = 0;
#endif

uint8_t  eeEnableOscillator  EEMEM = 1;

static void LoadEEPROM(void)
{
    Fosc = eeprom_read_dword(&eeFosc);
    Fout = eeprom_read_dword(&eeFout);

#if TAMER_VER < 200
#ifndef VCO_FIXED
    VCO_MIN = eeprom_read_word(&eeVCO_MIN);
    VCO_MAX = eeprom_read_word(&eeVCO_MAX);
    VCO_Kbit = eeprom_read_word(&eeVCO_Kbit);
#endif
    LMK_OutMask = eeprom_read_byte(&eeLMK_OutMask);
#endif

    AutoFreq = eeprom_read_byte(&eeAutoFreq);

#ifdef PRESENT_DAC12
    DacValue = eeprom_read_word(&eeDacValue);
#endif

#ifdef PRESENT_GPS
    AutoUpdateGps = eeprom_read_byte(&eeAutoGPSSync);
#endif

#if TAMER_VER >= 12 && TAMER_VER < 200
    EnableOscillator = eeprom_read_byte(&eeEnableOscillator);
#endif
}

static void StoreEEPROM_adf4355_reg(int32_t reg)
{
    uint8_t regno = reg & 0xf;
    if (regno > 12)
        return; // Invalid value, ignoring

    eeprom_write_dword(&ee_adf4355_regs[regno], reg);
}

static int32_t LoadEEPROM_adf4355_reg(uint8_t regno)
{
    if (regno > 12)
        return -1; // Invalid value, ignoring

    return eeprom_read_dword(&ee_adf4355_regs[regno]);
}

static void StoreEEPROM(void)
{
    eeprom_write_dword(&eeFosc, Fosc);
    eeprom_write_dword(&eeFout, Fout);

#if TAMER_VER < 200
#ifndef VCO_FIXED
    eeprom_write_word(&eeVCO_MIN, VCO_MIN);
    eeprom_write_word(&eeVCO_MAX, VCO_MAX);
    eeprom_write_word(&eeVCO_Kbit, VCO_Kbit);
#endif
    eeprom_write_byte(&eeLMK_OutMask, LMK_OutMask);
#endif

    eeprom_write_byte(&eeAutoFreq, AutoFreq);

#ifdef PRESENT_DAC12
    eeprom_write_word(&eeDacValue, DacValue);
#endif

#ifdef PRESENT_GPS
    eeprom_write_byte(&eeAutoGPSSync, AutoUpdateGps);
#endif

#if TAMER_VER >= 12 && TAMER_VER < 200
    eeprom_write_byte(&eeEnableOscillator, EnableOscillator);
#endif

}

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
    FillResultNoNewLinePM(newLine);
}


extern uint8_t gpsmode;
extern uint8_t commands;
extern RingBuff_t USBtoUSART_Buffer;
//extern RingBuff_t USARTtoUSB_Buffer;

static uint8_t SetDac(uint16_t value)
{
    if (value < 0x1000)
    {
        DAC12_WRITE(value);
        return 1;
    }

    return 0;
}

void AutoStartControl(void)
{
    DacSyncInit();
    DacSyncSet();

#ifdef PRESENT_GPS
    //InitCounters();
#endif

    AutoFreq = eeprom_read_byte(&eeAutoFreq);
    if (AutoFreq)
    {
        LoadEEPROM();

#ifdef PRESENT_DAC12
        SetDac(DacValue);
#endif

        int8_t i;
        union {
            uint8_t          data[4];
            uint32_t         u32data;
        } v;

        for (i = 12; i >= 0; --i) {
            v.u32data = LoadEEPROM_adf4355_reg(i);
            if ((v.data[0] & 0xf) != i)
                return; //Failed init

            write_reg_ADF4355(v.data[3], v.data[2], v.data[1], v.data[0]);
        }

        _delay_ms(50);

        // Tune procedure
        v.u32data = LoadEEPROM_adf4355_reg(4) | ((uint32_t)1<<REG4_CNTR_RESET);
        write_reg_ADF4355(v.data[3], v.data[2], v.data[1], v.data[0]);

        v.u32data = LoadEEPROM_adf4355_reg(2);
        write_reg_ADF4355(v.data[3], v.data[2], v.data[1], v.data[0]);

        v.u32data = LoadEEPROM_adf4355_reg(1);
        write_reg_ADF4355(v.data[3], v.data[2], v.data[1], v.data[0]);

        v.u32data = LoadEEPROM_adf4355_reg(0) & (~((uint32_t)1<<REG0_AUTOCAL_SHIFT));
        write_reg_ADF4355(v.data[3], v.data[2], v.data[1], v.data[0]);

        v.u32data = LoadEEPROM_adf4355_reg(4) & (~((uint32_t)1<<REG4_CNTR_RESET));
        write_reg_ADF4355(v.data[3], v.data[2], v.data[1], v.data[0]);

        _delay_ms(10);

        v.u32data = LoadEEPROM_adf4355_reg(0) | ((uint32_t)1<<REG0_AUTOCAL_SHIFT);
        write_reg_ADF4355(v.data[3], v.data[2], v.data[1], v.data[0]);
    }

}

uint8_t SetOutFreq(void)
{


    return 0;
}

uint8_t ProcessCommand(void)
{
    switch(command.cmd)
    {
        case cmdIDLE:
            return 1;

#if (TAMER_VER >= 122) && defined (PRESENT_GPS)
        case cmdGPSMODE:
    	    gpsmode = gpsmode ^ 1;
    	    return 1;
#endif

#ifndef NO_CMDREG
        case cmdREGISTER:
        {
            switch (command.type)
            {
                case trgDAC:
                    switch (command.details)
                    {
                        case detD12:
                            write_reg_DAC12(command.data[1], command.data[0]);
                            FillResultPM(resOk);
                            return 1;
                        default:
                            return 0;
                    }
                case trgADF:
                    write_reg_ADF4355(command.data[3],
                                      command.data[2],
                                      command.data[1],
                                      command.data[0]);
                    FillResultPM(resOk);
                    return 1;
                    break;
                case trgJTG:
                    switch (command.details) 
                    {
                    case detR00: 
                        JTAGReset();
                        JTAGSIR(0x2CCUL, 10);
                        JTAGRunTest(1003);
                        JTAGSIR(0x203UL, 10);
                        JTAGRunTest(8);
                        JTAGSDR(0x0089UL, 13);
                        JTAGSIR(0x205UL, 10);
                        JTAGRunTest(8);
                        FillUint16Hex(JTAGSDR(0xffffUL, 16));
                        FillUint16Hex(JTAGSDR(0xffffUL, 16));
                        FillUint16Hex(JTAGSDR(0xffffUL, 16));
                        FillUint16Hex(JTAGSDR(0xffffUL, 16));
                        FillUint16Hex(JTAGSDR(0xffffUL, 16));
                        FillResultNoNewLinePM(newLine);
                        return 1;
                    case detRST: JTAGReset(); FillResultPM(resOk); return 1;
                    case detRUN: JTAGRunTest(command.u32data); FillResultPM(resOk); return 1;
                    case detSDR: FillCmd();  FillUint32(JTAGSDR((uint32_t)command.u16data[0], command.data[3]));      FillResultNoNewLinePM(newLine); return 1;
                    case detSIR: FillCmd();  FillUint32(JTAGSIR((uint32_t)command.u16data[0], command.data[3]));      FillResultNoNewLinePM(newLine); return 1;
                    default:
                        return 0;
                    }
                    break;
                case trgPLD:
                    FillCmd();
                    FillUint32(write_reg_CPLD(command.u32data));
                    FillResultNoNewLinePM(newLine);
                    return 1;
                default:
                    return 0;
            }
            break;
        }
#endif
#ifndef NO_CMDPIN
        case cmdPIN:
        {
            switch (command.type)
            {
                case trgLED:
                {
                    if (command.data[0])
                        LedSet();
                    else
                        LedClear();

                    FillResultPM(resOk);
                    return 1;
                }
                case trgADF:
                {
                    if (command.details == detDIVIDERS) {
                        if (command.data[0])
                            SyntSelSet();
                        else
                            SyntSelClear();

                        FillResultPM(resOk);
                        return 1;
                    }
                    return 0;
                }
                default:
                    return 0;
            }
            return 0;
        }
#endif
        case cmdSET:
        {
            switch (command.type)
            {
                case trgNONE:
                {
                    switch (command.details)
                    {
                        case trgNONE:
                            break;
                        case detOUT:
                            Fout = command.u32data;
                            break;
                        default:
                            return 0;
                    }

                    uint8_t r = SetOutFreq();
                    if (r) {
                        FillResultPM(resOk);
                    } else {
                        FillResultPM(resBadRange);
                    }
                    return 1;
                }
#if TAMER_VER >= 12 && TAMER_VER < 200
                case trgIOS:
                {
                    if (command.details == detEN)
                    {
                        if (command.data[0])
                            EnableOscillator = 1;
                        else
                            EnableOscillator = 0;

                        SetOscillator();
                        FillResultPM(resOk);
                        return 1;
                    }
                    return 0;
                }
#endif
#ifdef PRESENT_DAC12
                case trgDAC:
                    switch (command.details)
                    {
                        case detNONE:
                            break;

                        case detD12:
                            DacValue = command.u16data[0];
                            break;

                        default:
                            return 0;
                    }

                    if (SetDac(DacValue))
                        FillResultPM(resOk);
                    else
                        FillResultPM(resBadRange);
                    return 1;
#endif
#ifdef PRESENT_GPS
                case trgGPS:
                {
                    switch (command.details)
                    {
#if TAMER_VER < 200
                        case detSYN:    UpdateOSCValue(); break;
#endif
                        case detAUTO:   AutoUpdateGps = command.data[0]; break;
                        default:
                            return 0;
                    }

                    FillResultPM(resOk);
                    return 1;
                }
#endif
#if defined(SELF_TESTING) && TAMER_VER < 200
                case trgSTS:
                {
                    switch (command.details)
                    {
                    case detSYN:
                        FillCmd();
                        FillUint16(SelfTestLockPin());
                        FillResultNoNewLinePM(newLine);
                        return 1;
                    case detAUTO:
                        SelfTestFull();
                        return 1;
                    default:
                        if (command.data[0] == 0)
                            SelfTestStop();
                        else if (command.data[0] < 128)
                            SelfTestStart(command.data[0]);
                        else {
                            FillResultPM(resBadRange);
                            return 1;
                        }
                    }

                    FillResultPM(resOk);
                    return 1;
                }
#endif
                default:
                    return 0;
            }
            return 0;
        }
#ifndef NO_CMDINFO
        case cmdINFO:
        {
            switch (command.type)
            {
#if TAMER_VER >= 12 && TAMER_VER < 200
                case trgIOS:
                {
                    if (command.details == detEN)
                    {
                        FillCmd();  FillUint16(EnableOscillator); FillResultNoNewLinePM(newLine); return 1;
                    }
                    return 0;
                }
#endif
                case trgNONE:
                {
                    switch (command.details)
                    {
                        case detOUT:   FillCmd();  FillUint32(Fout);      FillResultNoNewLinePM(newLine); break;
                        case detOSC:   FillCmd();  FillUint32(Fosc);      FillResultNoNewLinePM(newLine); break;
                        case detAUTO:  FillCmd();  FillUint16(AutoFreq);  FillResultNoNewLinePM(newLine); break;
                        default: return 0;
                    }
                    return 1;
                }

#ifdef PRESENT_DAC12
                case trgDAC:  FillCmd();  FillUint16(DacValue);  FillResultNoNewLinePM(newLine); return 1;
#endif
                case trgADF:
                {
                    switch (command.details)
                    {
                        case detLCK: FillCmd();  FillUint16(IsVcoLocked()); FillResultNoNewLinePM(newLine); break;
                        default: return 0;
                    }
                    return 1;
                }
                default:
                  return 0;
            }

            return 0;
        }
#endif
        case cmdRESET:
        {
            FillResultPM(resOk);
            return 1;
        }

#ifndef NO_HWINFO
        case cmdHWINFO:
            LoadHwInfo();
            return 1;
#endif

#ifndef NO_VERSION
        case cmdVERSION:
            FillResultPM(resVersion);
            return 1;
#endif

#ifndef NO_CMDEELOAD
        case cmdLOAD_EEPROM:
            switch (command.type)
            {
            case trgADF:
                FillCmd(); FillUint32(LoadEEPROM_adf4355_reg(command.data[0])); FillResultNoNewLinePM(newLine); return 1;
            default:
                LoadEEPROM();
                FillResultPM(resOk);
            }
            return 1;
#endif

        case cmdSTORE_EEPROM:
            switch (command.type)
            {
            case trgADF:
                StoreEEPROM_adf4355_reg(command.u32data);
                break;
            default:
                StoreEEPROM();
            }

            FillResultPM(resOk);
            return 1;

        default:
            return 0;

    }
	return 0;
}
