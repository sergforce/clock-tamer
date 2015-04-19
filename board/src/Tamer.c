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
 *  Main source file for the Tamer project. This file contains the main tasks of
 *  the demo and is responsible for the initial application hardware configuration.
 */

#include "Tamer.h"
#include "TamerControl.h"
#include <avr/power.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>

#define SPI_ENABLED


/** Circular buffer to hold data from the host before it is sent to the device via the serial port. */
RingBuff_t USBtoUSART_Buffer;

/** Circular buffer to hold data from the serial port before it is sent to the host. */
RingBuff_t USARTtoUSB_Buffer;

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
{
    .Config =
    {
        .ControlInterfaceNumber         = 0,
        .DataINEndpoint                 =
        {
            .Address          = CDC_TX_EPADDR,
            .Size             = CDC_TXRX_EPSIZE,
            .Banks            = 1,
        },
        .DataOUTEndpoint =
        {
            .Address          = CDC_RX_EPADDR,
            .Size             = CDC_TXRX_EPSIZE,
            .Banks            = 1,
        },
        .NotificationEndpoint =
        {
            .Address          = CDC_NOTIFICATION_EPADDR,
            .Size             = CDC_NOTIFICATION_EPSIZE,
            .Banks            = 1,
        },
    },
};



/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */

void FillUint16(uint16_t val);
void FillResultPM(const uint8_t* res);



const uint8_t resSyntax[] PROGMEM = "SYNTAX ERROR";
const uint8_t resErr[] PROGMEM = "CMD ERROR";


volatile uint8_t commands = 0;
#if (TAMER_VER >= 122)
uint8_t gpsmode = 0;
#endif

#if TAMER_VER >= 12 && TAMER_VER < 200
void SetOscillatorMode(uint8_t);
#endif


#define USART_BAUD 9600ul
#define USART_UBBR_VALUE ((F_CPU/(USART_BAUD<<4))-1)

void USART_vInit(void)
{
    // Set baud rate
    UBRR1H = (uint8_t)(USART_UBBR_VALUE>>8);
    UBRR1L = (uint8_t)USART_UBBR_VALUE;

    // Set frame format to 8 data bits, no parity, 1 stop bit
    UCSR1C = (0<<USBS1)|(1<<UCSZ11)|(1<<UCSZ10);
    // Enable receiver and transmitter
    UCSR1B = (1<<RXEN1)|(1<<TXEN1);
}

void DoExtraTasks(uint8_t dosend)
{
    if (dosend && USB_DeviceState == DEVICE_STATE_Configured)
    {
        /* Read bytes from the USART receive buffer into the USB IN endpoint */
        while (USARTtoUSB_Buffer.Elements)
            CDC_Device_SendByte(&VirtualSerial_CDC_Interface, Buffer_GetElement(&USARTtoUSB_Buffer));
    }
    else if (dosend && USB_DeviceState != DEVICE_STATE_Configured)
    {
        _delay_ms(100); //wait to flush spi buffer
    }

    CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
    USB_USBTask();
}


int main(void)
{
#if TAMER_VER >= 12 && TAMER_VER < 200
    SetOscillatorMode(0);
#endif

    INFOLED_DDR |=  (1 << INFOLED);
    INFOLED_PORT |= (1 << INFOLED);

    SetupHardware();
    BoardInit();

    AutoStartControl();

    Buffer_Initialize(&USBtoUSART_Buffer);
    Buffer_Initialize(&USARTtoUSB_Buffer);


    USART_vInit();

    sei();

    for (;;)
    {
#if (TAMER_VER >= 122) && defined (PRESENT_GPS)
restart_cycle:

        if (gpsmode)
        {
            for (uint8_t DataBytesRem = CDC_Device_BytesReceived(&VirtualSerial_CDC_Interface); DataBytesRem != 0; DataBytesRem--)
            {
                uint8_t byte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

                if (byte == '%')
                {
                    gpsmode = 0;
                    goto restart_cycle;
                }

                while((UCSR1A&(1<<UDRE1)) == 0);
                // Transmit data
                UDR1 = byte;
            }

            if (USB_DeviceState == DEVICE_STATE_Configured)
            {
                uint8_t byte;
                uint16_t timeout = 1000;

                // Wait until a byte has been received
                while((UCSR1A&(1<<RXC1)) == 0)
                {
                    if (--timeout == 0)
                        goto skip_data_send;
                }
                // Return received data
                byte = UDR1;

                /* Read bytes from the USART receive buffer into the USB IN endpoint */
                CDC_Device_SendByte(&VirtualSerial_CDC_Interface, byte);
            }

skip_data_send:
#if TAMER_VER < 200
            TamerControlAux();
#endif

            CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
            USB_USBTask();
        }
        else
#endif
        {

            for (uint8_t DataBytesRem = CDC_Device_BytesReceived(&VirtualSerial_CDC_Interface); DataBytesRem != 0; DataBytesRem--)
            {
                if (!(BUFF_STATICSIZE - USBtoUSART_Buffer.Elements))
                    break;

                uint8_t byte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

                if ((byte != 0) && (byte != 0xff))
                    Buffer_StoreElement(&USBtoUSART_Buffer, byte);


                // Uncomment this to enable echo on console
                // CDC_Device_SendByte(&VirtualSerial_CDC_Interface, byte);

                if (byte == '\n' || byte == '\r')
                {
                    commands++;
                    break;
                }
            }

            // Clean up buffer if it's full and there're no commands
            if ((commands == 0) && (!(BUFF_STATICSIZE - USBtoUSART_Buffer.Elements)))
                Buffer_GetElement(&USBtoUSART_Buffer);


            for (;commands>0;commands--)
            {
                uint8_t res = ParseCommand();
                if (res)
                {
                    res = ProcessCommand();
                    if (res == 0)
                    {
                        FillResultPM(resErr);
                    }
                }
                else
                {
                    FillResultPM(resSyntax);
                }
            }


            if (USB_DeviceState == DEVICE_STATE_Configured)
            {
                /* Read bytes from the USART receive buffer into the USB IN endpoint */
                while (USARTtoUSB_Buffer.Elements)
                    CDC_Device_SendByte(&VirtualSerial_CDC_Interface, Buffer_GetElement(&USARTtoUSB_Buffer));
            }

#if TAMER_VER < 200
            TamerControlAux();
#endif

            /* Load bytes from the USART transmit buffer into the USART */
            //if (USBtoUSART_Buffer.Elements)
            //  Serial_TxByte(Buffer_GetElement(&USBtoUSART_Buffer));

            //Serial_TxByte(CDC_Device_SendByte(&VirtualSerial_CDC_Interface, '\n'));

            CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
            USB_USBTask();
        }
    }
}


/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
    /* Disable watchdog if enabled by bootloader/fuses */
    MCUSR &= ~(1 << WDRF);
    wdt_disable();

    /* Disable clock division */
    clock_prescale_set(clock_div_1);

#if (!defined(FIXED_CONTROL_ENDPOINT_SIZE))
    USB_Device_ControlEndpointSize = CDC_CONTROL_ENDPOINT_SIZE;
#endif

    USB_Init();

#if defined(SPI_ENABLED) && (TAMER_VER < 200)
    //Enable MISO
    DDRB = (1<<PB3);

    SPCR = (1<<SPIE) | (1<<SPE) | (1<<CPOL);
    //SPDR = 0xff;
#endif

}

/** Event handler for the library USB Unhandled Control Request event. */
void EVENT_USB_Device_ControlRequest(void)
{
    CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
    if (!(CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface)))
        LedClear();
    else
        VirtualSerial_CDC_Interface.State.LineEncoding.BaudRateBPS = 9600;
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
    LedSet();
#if defined(SPI_ENABLED) && (TAMER_VER < 200)
    SPCR &=~(1<<SPE);
#endif
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
    LedClear();
#if defined(SPI_ENABLED) && (TAMER_VER < 200)
    SPCR |= (1<<SPE);
#endif
}



