//###########################################################################
//
// FILE:    sci_ex1_loopback.c
//
// TITLE:   SCI FIFO Digital Loop Back.
//
//! \addtogroup driver_example_list
//! <h1>SCI FIFO Digital Loop Back</h1>
//!
//!  This program uses the internal loop back test mode of the peripheral.
//!  Other then boot mode pin configuration, no other hardware configuration
//!  is required. The pinmux and SCI modules are configured through the 
//!  sysconfig file.
//!
//!  This test uses the loopback test mode of the SCI module to send
//!  characters starting with 0x00 through 0xFF.  The test will send
//!  a character and then check the receive buffer for a correct match.
//!
//!  \b Watch \b Variables \n
//!  - \b loopCount - Number of characters sent
//!  - \b errorCount - Number of errors detected
//!  - \b sendChar - Character sent
//!  - \b receivedChar - Character received
//!
//
//#############################################################################
//
// $Release Date: $
// $Copyright:
// Copyright (C) 2013-2023 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"

//
// Globals
//
uint16_t loopCount;
uint16_t errorCount;

//
// Function Prototypes
//
void error();
void intToStrPositive(uint32_t value, char* str)
{
    uint32_t i = 0;

    // Convert each digit to a character
    do {
        str[i++] = (char)(value % 10) + '0';
        value /= 10;
    } while (value > 0);

    // Reverse the string
    uint32_t j;
    for (j = 0; j < i / 2; j++) {
        char temp = str[j];
        str[j] = str[i - j - 1];
        str[i - j - 1] = temp;
    }

    // Null-terminate the string
    str[i] = '\0';
}

//
// Main
//
#define ENCODER_RESOLUTION 4095    // Number of counts per revolution for the encoder
int32_t pos=0;
// Function to initialize eQEP module for encoder position calculation
void initEQEP(void)
{
    //
    // Configure the decoder for quadrature count mode
    //
    EQEP_setDecoderConfig(EQEP1_BASE, (EQEP_CONFIG_1X_RESOLUTION |
                                       EQEP_CONFIG_QUADRATURE |
                                       EQEP_CONFIG_NO_SWAP));
    EQEP_setEmulationMode(EQEP1_BASE, EQEP_EMULATIONMODE_RUNFREE);

    //
    // Configure the position counter to reset on an index event
    //
    EQEP_setPositionCounterConfig(EQEP1_BASE, EQEP_POSITION_RESET_IDX,
                                  0xFFFFFFFF);

    //
    // Enable the unit timer, setting the frequency to 100 Hz
    //
    EQEP_enableUnitTimer(EQEP1_BASE, (DEVICE_SYSCLK_FREQ / 100));

    //
    // Configure the position counter to be latched on a unit time out
    //
    EQEP_setLatchMode(EQEP1_BASE, EQEP_LATCH_UNIT_TIME_OUT);

    //
    // Enable the eQEP1 module
    //
    EQEP_enableModule(EQEP1_BASE);

    //
    // Configure and enable the edge-capture unit. The capture clock divider is
    // SYSCLKOUT/64. The unit-position event divider is QCLK/32.
    //
    EQEP_setCaptureConfig(EQEP1_BASE, EQEP_CAPTURE_CLK_DIV_64,
                          EQEP_UNIT_POS_EVNT_DIV_32);
    EQEP_enableCapture(EQEP1_BASE);
}
int32_t getEncoderPosition(void)
{
    uint32_t position = EQEP_getPosition(EQEP1_BASE);
    return (int32_t)(position / (ENCODER_RESOLUTION / 360));
}
void main(void)
{



    Device_init();

    Device_initGPIO();


    Interrupt_initModule();


    Interrupt_initVectorTable();


    Board_init();
    GPIO_setPinConfig(GPIO_20_EQEP1A);
        GPIO_setPadConfig(20, GPIO_PIN_TYPE_STD);

        GPIO_setPinConfig(GPIO_21_EQEP1B);
        GPIO_setPadConfig(21, GPIO_PIN_TYPE_STD);

        GPIO_setPinConfig(GPIO_23_EQEP1I);
        GPIO_setPadConfig(23, GPIO_PIN_TYPE_STD);

        EINT;
           ERTM;
           initEQEP();
//    char a[16]={};
//    intToStrPositive(123456789,a);
//
//
//    SCI_writeCharArray(mySCI0_BASE, (uint16_t *)a, sizeof(a));
//    SCI_writeCharArray(mySCI0_BASE, (uint16_t *)("\r\n"), sizeof("\r\n"));

       int32_t position=EQEP_getPosition(EQEP1_BASE)/360;
       while (1)
       {
           // Get current position of the encoder
//           int32_t encoderPos = getEncoderPosition();
//           char b[32]={};
//           intToStrPositive(encoderPos,b);
//           SCI_writeCharArray(mySCI0_BASE, (uint16_t *)("ENCODER POSITION="), sizeof("ENCODER POSITION="));
//           SCI_writeCharArray(mySCI0_BASE, (uint16_t *)b, sizeof(b));
//           SCI_writeCharArray(mySCI0_BASE, (uint16_t *)("\r\n"), sizeof("\r\n"));
//           SCI_writeCharArray(mySCI0_BASE, (uint16_t *)("Direct counter value ="), sizeof("Direct counter value ="));
//
//           char c[32]={};
//           intToStrPositive(EQEP_getPosition(EQEP1_BASE), c);
//           SCI_writeCharArray(mySCI0_BASE, (uint16_t *)c, sizeof(c));
//           SCI_writeCharArray(mySCI0_BASE, (uint16_t *)("\r\n"), sizeof("\r\n"));

           int16_t m;
           if (EQEP_getPosition(EQEP1_BASE)>4096)
              {
                  m= EQEP_getPosition(EQEP1_BASE)%4096;
              }
           else
           {
               m= EQEP_getPosition(EQEP1_BASE);
           }
           char l[16]={};
           SCI_writeCharArray(mySCI0_BASE, (uint16_t *)("COUNTER RESET="), sizeof("COUNTER RESET="));
             intToStrPositive(m, l);
             SCI_writeCharArray(mySCI0_BASE, (uint16_t *)l, sizeof(l));
             SCI_writeCharArray(mySCI0_BASE, (uint16_t *)("\r\n"), sizeof("\r\n"));
//             SCI_writeCharArray(mySCI0_BASE, (uint16_t *)("ANGLE VALUE="), sizeof("ANGLE VALUE="));

//           char d[32]={};
//           float32_t t=(m/4095);
//           int32_t k=t;
//           intToStrPositive((k), d);
//           SCI_writeCharArray(mySCI0_BASE, (uint32_t *)d, sizeof(d));
//           SCI_writeCharArray(mySCI0_BASE, (uint16_t *)("\r\n"), sizeof("\r\n"));
           // Delay or perform other tasks
       }
}










//
// End of file
//
