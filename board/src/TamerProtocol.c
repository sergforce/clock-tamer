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

// gcc TamerProtocol.c -g -Wall -o TamerTest -D_SELF_TEST_


#include <stdint.h>
#ifndef _SELF_TEST_
#include "Tamer.h"
#else
#include <stdio.h>
#include <string.h>
#endif
#include "TamerProtocol.h"

#ifdef _SELF_TEST_

#define PROGMEM
uint8_t getbyte(void);
uint8_t pgm_read_byte(uint8_t* p);

#else

extern RingBuff_t USBtoUSART_Buffer;

#define getbyte()   Buffer_GetElement(&USBtoUSART_Buffer)

#endif

/* Static variables */
TamerCommand_t command;

/* Constants */
#define WORD_SIZE       3

const uint8_t newLine[] PROGMEM = "\r\n";

static const uint8_t pCmd[]  PROGMEM  =
        "REG"
        "PIN"
        "SET"
        "SAV"
        "DEF"
        "INF"
        "VER"
        "HWI"
        "RST"
        "LDE"
        "STE"
        "%%%";
#define CMD_COUNT    (sizeof(pCmd)  / WORD_SIZE)

static const uint8_t pTrg[]  PROGMEM  =
        "LMK"
        "LMX"
        "DAC"
        "LED"
        "VCO"
        "GPS"
        "IOS"
        "STS"
        "ADF"
        "JTG"
        "PLD";
#define TRG_COUNT    (sizeof(pTrg)  / WORD_SIZE)

static const uint8_t pDet[]  PROGMEM  =
        "ENB"
        "GOE"
        "SYN"
        "OSC"
        "OUT"
        "PRT"
        "MIN"
        "MAX"
        "KBT"
        "DIV"
        "AUT"
        "D12"
        "R00"
        "R01"
        "R02"
        "R03"
        "LCK"
        "RST"
        "RUN"
        "SDR"
        "SIR";
#define DET_COUNT    (sizeof(pDet)  / WORD_SIZE)




static inline uint8_t ParseParam(uint8_t w1, uint8_t w2, uint8_t w3, const uint8_t* table)
{
    uint8_t i = 1;

    for (;;i++)
    {
        uint8_t o1 = pgm_read_byte(table++);
        uint8_t o2 = pgm_read_byte(table++);
        uint8_t o3 = pgm_read_byte(table++);

        if ((o1 == 0) /*|| (o2 == 0) || (o3 == 0)*/)
            return 0xff;

        if ((o1 == w1) && (o2 == w2) && (o3 == w3))
            return i;
    }
}

static uint8_t ParseValueD(uint8_t w1)
{
    if (w1 >= '0' && w1 <= '9')
        return w1 - '0';

    return 0x10;
}

#ifndef NO_HEXVALUES
static uint8_t ParseValue(uint8_t w1)
{
    if (w1 >= '0' && w1 <= '9')
        return w1 - '0';
    if (w1 >= 'a' && w1 <= 'f')
        return 10 + w1 - 'a';
    if (w1 >= 'A' && w1 <= 'F')
        return 10 + w1 - 'A';

    return 0x10;
}
#endif

static uint8_t IsCommandSeparator(uint8_t byte)
{
	if (byte == 0 || byte == '\r' || byte == '\n')
    	return 1;

	return 0;
}

uint8_t ParseCommand(void)
{
    uint8_t byte;
    uint8_t byte2;
    uint8_t byte3;

    uint8_t step = 0;
    uint8_t val = 0;


    command.cmd = 0;
    command.type = 0;
    command.details = 0;
    command.u32data = 0;

    for (; step < 4; )
    {
        // Skip white spaces
        do
        {
            byte = getbyte();
        } while (byte == ' ');

        if (byte == ',')
        {
            step++;
            val = 0;
            continue;
        }

        if (IsCommandSeparator(byte))
            return 1;


        if (step < 3)
        {
            byte2 = getbyte();
            byte3 = getbyte();

			if (IsCommandSeparator(byte2) || IsCommandSeparator(byte3))
				return 0; 

            const uint8_t* mem;
            switch (step)
            {
                case 0: mem = pCmd; break;
                case 1: mem = pTrg; break;
                case 2: mem = pDet; break;
            }
            val = ParseParam(byte, byte2, byte3, mem);
            switch (step)
            {
                case 0: command.cmd = val;     break;
                case 1: command.type = val;    break;
                case 2: command.details = val; break;
            }
        }
        else
        {
            uint8_t b1;
            uint8_t j;

#ifndef NO_HEXVALUES
            if ((byte == 'x') || (byte == 'X'))
            {

              for (j = 0; j < 8; j++)
              {
                b1 = ParseValue((byte = getbyte()));
                if (b1 == 0x10)
                    goto skip;

                command.u32data = (command.u32data << 4) | b1;
              }
            }
            else
#endif
            {
              b1 = ParseValueD(byte);
              if (b1 == 0x10)
                    goto skip;
              command.u32data = b1;

              for (j = 1; j < 10; j++)
              {
                b1 = ParseValueD((byte = getbyte()));
                if (b1 == 0x10)
                    goto skip;

                command.u32data = (command.u32data * 10u) + b1;
              }
            }
            byte = getbyte();
        skip:
            if (IsCommandSeparator(byte))
                return 1;
            return 0;
         }
    }

    return 0;
}

static void FillResultNoNewLinePM(const uint8_t *res)
{
    uint8_t byte;
    while ((byte = pgm_read_byte(res++)))
    {
        Store(byte);
    }
}

void FillNewLine(void)
{
    FillResultNoNewLinePM(newLine);
}

void FillResultPM(const uint8_t* res)
{
    FillResultNoNewLinePM(res);
    FillResultNoNewLinePM(newLine);
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

void FillCmd(void)
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

void FillUint32(uint32_t val)
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

#ifdef USE_HEX_OUTPUT
void FillUint8Hex(uint8_t val)
{
    FillCharHex(val >> 4);
    FillCharHex(val & 0xf);
}

void FillUint16Hex(uint16_t val)
{
    union {
        uint16_t v;
        uint8_t d[2];
    } m;

    m.v = val;

    FillUint8Hex(m.d[1]);
    FillUint8Hex(m.d[0]);
}

void FillUint32Hex(uint32_t val)
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

#endif


#ifdef _SELF_TEST_


uint8_t* buffer;
int pos = 0;
int len;

uint8_t getbyte(void)
{
    if (pos < len)
    {
        printf("@%i", pos);
        return buffer[pos++];
    }

    return 0;
}

uint8_t pgm_read_byte(uint8_t* p)
{
    return *p;
}


int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        printf("usage %s string\n", argv[0]);
        return -1;
    }

    buffer = (uint8_t*)argv[1];
    len = strlen(argv[1]);

    printf(" Command \"%s\", len = %d\n", buffer, len);
    int j = 0;

    for (; j < 10; j++)
    {
        int i = ParseCommand();
        printf(" Res = %d \n", i);

        printf(" CMD     %x\n", command.cmd);
        printf(" TYPE    %x\n", command.type);
        printf(" DETAILS %x\n", command.details);
        printf(" DATA    %02x.%02x.%02x.%02x  [%u.%u ; %u]\n", command.data[0], command.data[1], command.data[2], command.data[3],
            command.u16data[0], command.u16data[1], command.u32data);

        if (i == 0)
          return 0;
    }
    printf("Interrupted!\n");
    return 0;
}


#endif


