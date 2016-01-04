/*******************************************************************************
  Class B Library Implementation Test File

  Summary:
    This file contains the test program for the 
    the Class B Safety Software Library on PIC32MX MCUs.
    
*******************************************************************************/
/*******************************************************************************
FileName:       ClassB_main.c
Processor:      PIC32MX
Compiler:       Microchip MPLAB® C32 v1.04 or higher

Copyright © 2008-2009 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
the accompanying software only when embedded on a Microchip microcontroller or
digital signal controller that is integrated into your product or third party product.  
 
If the accompanying software required your consent to the terms of Microchip's
click-wrap license agreement, then you should also refer to such license agreement
for additional information regarding your rights and obligations.
Your acceptance and/or use of this software constitutes your agreement to the terms
and conditions of this notice and applicable click-wrap license, if any.

You agree that you are solely responsible for testing the code and determining its suitability.
Microchip has no obligation to modify, test, certify, or support the code.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/

#include <plib.h>

#include <stdlib.h>


#include "ClassB/SSL_CpuRegisterTest.h"
#include "ClassB/SSL_PcTest.h"
#include "ClassB/SSL_CBram.h"
#include "ClassB/SSL_MarchB.h"
#include "ClassB/SSL_MarchC.h"
#include "ClassB/SSL_Flash_CRC.h"
#include "ClassB/SSL_ClockTest.h"
#include "ClassB/SSL_ClockTest_LineFreq.h"


/* CPU Configuration */
// High Speed Primary PLL Osc
// WDT disabled
// sysclock = 80MHz, peripheral clock =80MHz
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FWDTEN = OFF
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1
#pragma config FPBDIV = DIV_1

#define     SYSTEM_CLOCK        80000000

volatile struct  ClassB_Test_Flags {
    unsigned cpuRegister_TestResult : 1;
    unsigned programCounter_TestResult:1;
    unsigned checkerboardRam_TestResult:1;
    unsigned marchCRam_TestResult:1;
    unsigned marchCMinusRam_TestResult:1;
    unsigned marchCRamStack_TestResult:1;
    unsigned marchBRam_TestResult:1;
    unsigned flash_TestResult:1;
    unsigned clock_TestResult:1;
    unsigned clockLine_TestResult:1;
} testFlag;


int main(void){


    /* Configure the system pre-fetch and cache */
    SYSTEMConfig( SYSTEM_CLOCK, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    /* Enable Vectored Interrupts */
    INTEnableSystemMultiVectoredInt();


    /**********************************************************************************/
    /*                                  CPU REGISTER TEST                             */                              
    /**********************************************************************************/

    if (SSL_32bitsFamily_CPU_RegisterTest()==CPU_REGISTER_TEST_PASS)
    {
        // the CPU registers don't have stuck bits
        testFlag.cpuRegister_TestResult = 1;
    }
    else 
    {
        testFlag.cpuRegister_TestResult = 0;
    }

    /**********************************************************************************/
    /*                                  PROGRAM COUNTER TEST                          */
    /*  This requires a special linker script (elf32pic32mx.ld) to be added           */
    /*   as part of the project. See the description in SSL_PcTest.h                  */
    /**********************************************************************************/

    if (SSL_32bitsFamily_PCtest()==PC_TEST_PASS) 
    {
        // the PC register does not have stuck bits
        testFlag.programCounter_TestResult = 1;
    }
    else 
    {
        testFlag.programCounter_TestResult = 0; 
    }

    /**********************************************************************************/
    /*                                  RAM TESTS                                     */                              
    /**********************************************************************************/

    // Variables used for RAM Tests
    extern int  _stack[];                   // the address of the stack, as placed by the linker
    extern int  _min_stack_size[];          // size of the stack, as defined in the project
    extern char _data_begin[];              // the address of the data segment, as placed by the linker

    int     *ramTestStartAddress;           // start address for the test 
    int     ramTestSize;                    // size of the tested area, bytes

    
    /*************************/
    /* Checker Board RAM test*/
    /*************************/
    // We'll test 1KB chunk at the middle of the RAM
    // Note that this test is not destructive
    // The size of the RAM area to test has to be multiple of 64.
    // It has to NOT overlap the stack space!

    int         ramTotSize;        // total RAM available, without stack
    char*       ramStartAddress;   // starting RAM address for the test
    char*       ramEndAddress;     // end address of RAM

    ramStartAddress= (char*)_data_begin;
    ramEndAddress= (char*)(_stack-(int)_min_stack_size/4);      // we leave out the stack space

    ramTotSize=ramEndAddress-ramStartAddress;

    // check if we have at least 1 KB available
    ramTestSize = (ramTotSize>0x400)?0x400:ramTotSize;
    // make sure is a multiple of 64
    ramTotSize&=0xffffffc0;

    // select 1 KB memory chunk in the middle of the RAM
    ramTestStartAddress =(int*)(ramStartAddress+(ramTotSize-ramTestSize)/2);

    // test it
    if (SSL_32bitsFamily_RAMtest_CheckerBoard(ramTestStartAddress, ramTestSize)==CB_TEST_PASS)
    { 
        // the test succeeded and we are confident that the RAM area can be used
        testFlag.checkerboardRam_TestResult = 1;
    }
    else
    {
        testFlag.checkerboardRam_TestResult = 0;
    }


    /****************************/
    /*    March B Ram Test      */
    /****************************/
    // Suppose we need to allocate and want to use a 1 KB chunk of memory
    // We'll test it using the Checker Board test before we use it
    // to make sure there is no problem with the chunk of RAM we selected.
    // Note that this test is destructive.
    // Note that the size of the RAM to test has to be multiple of 4.

    ramTestSize = 1024;
    ramTestStartAddress = (int *) malloc(ramTestSize);  // allocate RAM memory for usage

    if(ramTestStartAddress)
    { // allocation succeeded and we can perform the test
        if (SSL_32bitsFamily_RAMtest_MarchB(ramTestStartAddress, ramTestSize )==MARCHB_TEST_PASS)
        {
            // the test succeeded and we can go ahead and use the allocated memory
            testFlag.marchBRam_TestResult = 1;
        }
        else
        {
            testFlag.marchBRam_TestResult = 0;
        }
        
        free(ramTestStartAddress);  // we're done with this memory block
                                    // return it to the heap
    }


    /*************************/
    /*    MarchC RAM tests    */
    /*************************/
    // We allocate and want to use a 2 KB chunk of memory
    // We'll test it using the March C and March C Minus tests before we use it
    // Note that the size of the RAM to test has to be multiple of 4.
    // This test is destructive too.

    ramTestSize = 2048;
    ramTestStartAddress = (int *) malloc(ramTestSize);

    if(ramTestStartAddress)
    {   // allocation succeeded and we can perform the test
        if (SSL_32bitsFamily_RAMtest_MarchC(ramTestStartAddress, ramTestSize )==MARCHC_TEST_PASS)
        { 
            testFlag.marchCRam_TestResult = 1;
        }
        else
        {
            testFlag.marchCRam_TestResult = 0;
        }


        /**************************/
        /* MarchC Minus RAM test  */
        /**************************/

        if (SSL_32bitsFamily_RAMtest_MarchC_Minus(ramTestStartAddress, ramTestSize )==MARCHC_TEST_PASS)
        { 
            // if both tests succeeded we can go ahead and use the allocated memory
            testFlag.marchCMinusRam_TestResult = 1;
        }
        else
        {
            testFlag.marchCMinusRam_TestResult = 0;  
        }

        free(ramTestStartAddress);  // we're done with this memory block
                                    // return it to the heap
    }

    /****************************/
    /* MarchC RAM and Stack Test*/
    /****************************/
    // We allocate and want to use a 2 KB chunk of memory
    // We want to make sure that both the allocated RAM and
    // the stack space are ok.
    // We'll test it using the March C and Stack tests before we use it
    // Note that the size of the RAM to test has to be multiple of 4.
    // Also, the size of the tested RAM area has to be greater than the
    // size of the tested stack area. 
    // This test is destructive for the RAM area but preserves the Stack area.

    ramTestSize = 2048;
    ramTestStartAddress = (int *) malloc(ramTestSize);

    if(ramTestStartAddress)
    {   // allocation succeeded and we can perform the test
        // test 1024 bytes of Stack
        // we use the address of the stack, as placed by the linker

        if (SSL_32bitsFamily_RAM_STACKtest_MarchC(ramTestStartAddress, ramTestSize, _stack, 1024 )==MARCHC_TEST_PASS)
        { 
            // if the test succeeded we can go ahead and use the allocated memory
            // we're confident that both RAM and Stack area are ok!       
            testFlag.marchCRamStack_TestResult = 1;
        }
        else
        {
            testFlag.marchCRamStack_TestResult = 0;
        }

        free(ramTestStartAddress);  // we're done with this memory block
                                    // return it to the heap
    }


    /**********************************************************************************/
    /*                                  FLASH CRC TEST                                */                              
    /**********************************************************************************/
    // This function can be called at startup to generate the Reference checksum.
    // The same function can be called periodically and the generated checksum can be 
    // compared with the reference checksum.
    // If both are the same the "flash_TestResult" status bit can be set. 

    unsigned int flashCrcRef, flashCrc; // reference and current CRC values
    unsigned int crcSeed=0xffff;        // initial CRC register value
                                        // this is the recommended CRC seed
                                        // for checking properly long 0 streams.

    // calculate the CRC16 of the whole program flash (K0)      
    char* flashStartAddress = (char*)0x9d000000;       // fixed start K0 address on PIC32MX devices
    char* flashEndAddress =  flashStartAddress+BMXPFMSZ-1;  // size of the flash on this device
                                                            // the BMX register stores the Flash size for this part

    // first we calculate the reference Flash CRC value
    flashCrcRef = SSL_32bitsFamily_Flashtest_CRC16( flashStartAddress, flashEndAddress, CRC_16_GEN_POLY, crcSeed);

    // at some time later we calculate again the CRC of the Flash 
    flashCrc = SSL_32bitsFamily_Flashtest_CRC16( flashStartAddress, flashEndAddress, CRC_16_GEN_POLY, crcSeed);

    // make sure that the periodic check is equal to the reference one
    if ( flashCrc==flashCrcRef)
    {
        testFlag.flash_TestResult=1;
        // we are confident that the data programmed in Flash
        // has not been altered in any way 
    }
    else
    { 
        testFlag.flash_TestResult=0;
    }


    /**********************************************************************************/
    /*        CLOCK  TEST WITH SECONDARY OSCILLATOR AS REFERENCE CLOCK                */
    /*      This test requires that a standard 32.768 KHz crystal is connected        */
    /*                      at the SOSC input.                                        */
    /**********************************************************************************/

    // we'll count for 1 second
    // check the system clock to be within +/- 0.1%

    if(SSL_32bitsFamily_CLOCKtest(SYSTEM_CLOCK, 1000, SYSTEM_CLOCK/1000, SYSTEM_CLOCK/1000)==CLOCK_TEST_PASS)
    {
        // the CPU clock is within the required limits
        testFlag.clock_TestResult=1;
    }
    else
    { 
        testFlag.clock_TestResult=0;
    }

    /**********************************************************************************/
    /*      CLOCK TEST WITH 50Hz LINE FREQUENCY AS REFERENCE CLOCK                    */
    /*      This test requires an 50 Hz external reference frequency to be fed        */
    /*                      to the IC1 input pin.                                     */    
    /**********************************************************************************/

    // we test using 50 Hz reference frequency
    // check the system clock to be within +/- 0.1%

    if(SSL_32bitsFamily_CLOCKtest_LineFreq(SYSTEM_CLOCK, 50, SYSTEM_CLOCK/1000, SYSTEM_CLOCK/1000)==CLOCK_TEST_PASS)
    { 
        // the CPU clock is within the required limits
        testFlag.clockLine_TestResult=1;
    }
    else
    { 
        testFlag.clockLine_TestResult=0; 
    }


    /* End of Tests */    

    while(1);

    return 0;


}



