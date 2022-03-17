//*****************************************************************************
//
// uart_echo.c - Example for reading data from and writing data to the UART in
//               an interrupt driven fashion.
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.1.4.178 of the EK-TM4C129EXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc\hw_uart.h"
#include "utils/uartstdio.h"

#include <math.h>

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART Echo (uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the USB debug virtual serial port on the evaluation board)
//! will be configured in 115,200 baud, 8-n-1 mode.  All characters received on
//! the UART are transmitted back to the UART.
//
//*****************************************************************************

//****************************************************************************
//
// System clock rate in Hz.
//
//****************************************************************************
uint32_t g_ui32SysClock;
uint8_t ban = 0;
uint8_t mensaje[];
uint8_t mensaje2[];
uint8_t vacio[];
uint32_t ind = 0;
//char mensaje2[];
uint8_t ban1 = 0;
uint8_t ban2 = 0;
uint8_t ban3 = 0;
uint8_t ban4 = 0;

uint8_t length = 0;
uint16_t length2 = 0;
int mensajes = 0;
//uint8_t* p;
int lvl = 0;
uint16_t kp1 = 3;
uint16_t ki1 = 1;
uint16_t kd1 = 1;
uint16_t kp2 = 5;
uint16_t ki2 = 4;
uint16_t kd2 = 7;
float kp;
float ki;
float kd;
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
    }
}
//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler(void)
{
    uint32_t ui32Status;
    char cThisChar;
    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART0_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //

        cThisChar = UARTCharGet(UART0_BASE);
        if (cThisChar == '@') {
            ban = 1;
        //ROM_UARTCharPutNonBlocking(UART0_BASE, cThisChar);
        }
        if (cThisChar == '!') {
            ban = 0;
        }
        if (ban == 1) {
            if (ind >= 1) {
                mensaje[ind] = cThisChar;
                if (ind >= 2) {
                    mensaje2[ind-1]=cThisChar;
                }
            }
            ind++;
        }

        if (ban == 0 && ind >= 1 ) {
            mensaje[ind] = '_';
            ind += 1;
            mensaje[ind] = 1;
            uint8_t* p = mensaje +2;
            mensajes = 0;
            mensajes = atoi(p);
            //length = sizeof(p)/sizeof(p[1]);
            length = ind-2;
            length2 = pow(10,length);
            //  mensaje2 = (char*)mensaje;
            //UARTSend((uint8_t *)p, ind-2);
            //UARTprintf("%d\n",mensajes);
            if (mensaje[1] == 'a') {
                ban1 = 1;
            }
            else if (mensaje[1] == 'b') {
                ban1 = 2;
            }
            else if (mensaje[1] == 'c') {
                ban1 = 3;
            }
            else if (mensaje[1] == 'd') {
                ban1 = 4;
            }
            else if (mensaje[1] == 'e') {
                ban1 = 5;
            }
            else if (mensaje[1] == 'f') {
                ban1 = 6;
            }
            else if (mensaje[1] == 'g') {
                ban1 = 7;
            }
            else {
                ban1 = 0;
            }
            //UARTSend((uint8_t *)mensaje, ind+1);
            ind = 0;
           // mensaje[1] = 0;
        }
        //
        // Blink the LED to show a character transfer is occuring.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        SysCtlDelay(ROM_SysCtlClockGet() / (1000 * 3));

        //
        // Turn off the LED
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
    }
}


//*****************************************************************************
//
// This example demonstrates how to send a string of data to the UART.
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the clocking to run directly from the crystal at 120MHz.
    //
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);
    // Enable the GPIO port that is used for the on-board LED.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PN0).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    //
    // Prompt for text to be entered.
    //
    UARTSend((uint8_t *)"\033[2JEnter text: ", 16);
    //UARTprintf("HOLA");
    //
    // Loop forever echoing data through the UART.
    //
    while(1)
    {
        switch(ban1){
        case 1:
            ban2 = 1;
            /*
            if ((int)mensajes>(int)0) {
                lvl = -mensajes;
            } else {
                lvl = -mensajes;
            }
            */
            lvl = mensajes;
            break;
        case 2:
            ban2 = 2;
            kp1 = mensajes;
            break;
        case 3:
            ban2 = 3;
            kp2 = mensajes;
            break;
        case 4:
            ban2 = 4;
            ki1 = mensajes;
            break;
        case 5:
            ban2 = 4;
            ki2 = mensajes;
            break;
        case 6:
            ban2 = 4;
            kd1 = mensajes;
            break;
        case 7:
            ban2 = 4;
            kd2 = mensajes;
            break;
        default:
            ban2 = 0;
            break;
        }
        kp = (float)kp2/(length2)+(float)kp1;
        ki = (float)ki2/1000+(float)ki1;
        kd = (float)kd2/1000+(float)kd1;
       // int i;
       // uint8_t* p = mensaje +2;
       // mensajes = atoi(p);
       // for (i = 1;i<length;i++){
            //mensaje = atoi()
       // }
    }
}
