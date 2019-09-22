//*****************************************************************************
//
// hello.c - Simple hello world example.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
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
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"


#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"


#include "inc/hw_gpio.h"

#include "inc/hw_timer.h"
#include "driverlib/timer.h"

#include "inc/hw_uart.h"

#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Hello World (hello)</h1>
//!
//! A very simple ``hello world'' example.  It simply displays ``Hello World!''
//! on the UART and is a starting point for more complicated applications.
//!
//! UART0, connected to the Virtual Serial Port and running at
//! 115,200, 8-N-1, is used to display messages from this application.
//
//*****************************************************************************

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
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************

uint32_t g_ui32Flags;


void
Timer1IntHandler(void)
{
    char cOne, cTwo;

    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the second timer.
    //
    HWREGBITW(&g_ui32Flags, 1) ^= 1;

    //
    // Use the flags to Toggle the LED for this timer
    //
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, g_ui32Flags << 1);

    //
    // Update the interrupt status on the display.
    //
    ROM_IntMasterDisable();
    cOne = HWREGBITW(&g_ui32Flags, 0) ? '1' : '0';
    cTwo = HWREGBITW(&g_ui32Flags, 1) ? '1' : '0';
 //   UARTprintf("\rT1: %c  T2: %c", cOne, cTwo);
    ROM_IntMasterEnable();
}

void
Timer0IntHandler(void)
{
    char cOne, cTwo;

    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the first timer.
    //
    HWREGBITW(&g_ui32Flags, 0) ^= 1;

    //
    // Use the flags to Toggle the LED for this timer
    //
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, g_ui32Flags << 1);

    //
    // Update the interrupt status on the display.
    //
    ROM_IntMasterDisable();
    cOne = HWREGBITW(&g_ui32Flags, 0) ? '1' : '0';
    cTwo = HWREGBITW(&g_ui32Flags, 1) ? '1' : '0';
   // UARTprintf("\rT1: %c  T2: %c", cOne, cTwo);
    ROM_IntMasterEnable();
}

void
ConfigureUART_1(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Enable UART1
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
    ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(1, 115200, 16000000);
}

//*****************************************************************************
//
// Print "Hello World!" to the UART on the evaluation board.
//
//*****************************************************************************


//******************************************************************************************


void
ConfigureUART_0(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART1
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}


void GetAndSendInitialTime(void)
{
    TimeRightNow=  ( UARTCharGet(UART0_BASE) - '0' ) ;


    TimeRightNow = ( TimeRightNow*10 ) + ( UARTCharGet(UART0_BASE) -'0') ;


    UARTCharPut(UART1_BASE,TimeRightNow);
}

void Timer0AndTimer1_Init(void)
{

    //
     // Enable the peripherals used by this example.
     //
     ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
     ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

     //
     // Enable processor interrupts.
     //
     ROM_IntMasterEnable();

     //
     // Set the clocking to run directly from the crystal.
     //
     ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                        SYSCTL_OSC_MAIN);

     //
     // Configure the two 32-bit periodic timers.
     //
     ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
     ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
     ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet());
     ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, ROM_SysCtlClockGet() / 2);

     //
     // Setup the interrupts for the timer timeouts.
     //
     ROM_IntEnable(INT_TIMER0A);
     ROM_IntEnable(INT_TIMER1A);
     ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
     ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
 //
 //
 //    //
 //    // Enable the timers.
 //    //
     ROM_TimerEnable(TIMER0_BASE, TIMER_A);
     ROM_TimerEnable(TIMER1_BASE, TIMER_A);

}

int
main(void)
{

    Timer0AndTimer1_Init();


        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

        HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
        HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
        HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
        GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
        GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    //
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF2 & PF3).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Initialize the UART.
    //


    ConfigureUART_1();

    ConfigureUART_0();

    int32_t  TimeRightNow=0;




    GetAndSendInitialTime();


    while(1)
    {

        if( (GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4) == 0)  &&  (GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0) == 0) )
        {


        }



        if ( GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4) == 0 )
        {
            while( GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4) == 0 );

            UARTCharPut(UART1_BASE,1);



            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

                   //
                   // Delay for a bit.
                   //
                   SysCtlDelay(SysCtlClockGet() / 10 / 3);

                   //
                   // Turn off the BLUE LED.
                   //
                   GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

                   //
                   // Delay for a bit.
                   //
                   SysCtlDelay(SysCtlClockGet() / 10 / 3);

        }
        else if ( GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0) == 0 )
        {
            while( GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0) == 0 );

                        UARTCharPut(UART1_BASE,5);

                        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

                           //
                           // Delay for a bit.
                           //
                           SysCtlDelay(SysCtlClockGet() / 10 / 3);

                           //
                           // Turn off the BLUE LED.
                           //
                           GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

                           //
                           // Delay for a bit.
                           //
                           SysCtlDelay(SysCtlClockGet() / 10 / 3);
        }
        else
        {

        }


    }
}
