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


const uint8_t resOk[] PROGMEM = "OK";
const uint8_t resBadRange[] PROGMEM = "Bad tuning range";


#if TAMER_VER >= 200
#include "TamerControl2.c"
#else
#include "TamerControl.c"
#endif



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

#ifdef PRESENT_DAC12
static uint8_t SetDac(uint16_t value)
{
    if (value < 0x1000)
    {
        DAC12_WRITE(value);
        return 1;
    }

    return 0;
}
#endif

// Common commands
static uint8_t OnCmdREG(void)
{
    switch (command.type)
    {
    case trgDAC:
        switch (command.details)
        {
#ifdef PRESENT_DAC12
        case detD12:
            write_reg_DAC12(command.data[1], command.data[0]);
            FillResultPM(resOk);
            return 1;
#endif
        default:
            return 0;
        }
    case trgADF: return OnCmdREG_ADF();
    case trgJTG: return OnCmdREG_JTG();
    case trgPLD: return OnCmdREG_PLD();

    case trgLMK: return OnCmdREG_LMK();
    case trgLMX: return OnCmdREG_LMX();

    default:     return 0;
    }
}

static uint8_t OnCmdPIN(void)
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
    case trgADF: return OnCmdPIN_ADF();
    case trgLMK: return OnCmdPIN_LMK();
    case trgLMX: return OnCmdPIN_LMX();
    default:     return 0;
    }
    return 0;
}


static uint8_t OnCmdSET(void)
{
    switch (command.type)
    {
        case trgNONE: return OnCmdSET_NONE();
        case trgIOS:  return OnCmdSET_IOS();
        case trgVCO:  return OnCmdSET_VCO();
        case trgLMK:  return OnCmdSET_LMK();
        case trgGPS:  return OnCmdSET_GPS();
        case trgSTS:  return OnCmdSET_STS();
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
                FillNewLine();
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

static uint8_t OnCmdNFO(void)
{
    switch (command.type)
    {
        case trgIOS:    return OnCmdNFO_IOS();
        case trgGPS:    return OnCmdNFO_GPS();
        case trgNONE:
        {
            switch (command.details)
            {
                case detOUT:   FillCmd();  FillUint32(Fout);      FillNewLine(); break;
                case detOSC:   FillCmd();  FillUint32(Fosc);      FillNewLine(); break;
                case detAUTO:  FillCmd();  FillUint16(AutoFreq);  FillNewLine(); break;
                default: return 0;
            }
            return 1;
        }

#ifdef PRESENT_DAC12
        case trgDAC:  FillCmd();  FillUint16(DacValue);  FillNewLine(); return 1;
#endif
        case trgADF:
        {
            switch (command.details)
            {
                case detLCK: FillCmd();  FillUint16(IsVcoLocked()); FillNewLine(); break;
                default: return 0;
            }
            return 1;
        }
        default:
          return 0;
    }

    return 0;
}


uint8_t ProcessCommand(void)
{
    switch (command.cmd) {
    case cmdIDLE:         return 1;
    case cmdGPSMODE:      return OnCmdGPS();
    case cmdREGISTER:     return OnCmdREG();
    case cmdPIN:          return OnCmdPIN();
    case cmdSET:          return OnCmdSET();
    case cmdINFO:         return OnCmdNFO();
    case cmdRESET:        FillResultPM(resOk);      return 1;
    case cmdHWINFO:       LoadHwInfo();             return 1;
    case cmdVERSION:      FillResultPM(resVersion); return 1;
    case cmdLOAD_EEPROM:  return OnCmdLDE();
    case cmdSTORE_EEPROM: return OnCmdSTE();
    default:              return 0;
    }
}

