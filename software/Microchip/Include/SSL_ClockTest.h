/*******************************************************************************
  Class B Library Interface Definition

  Summary:
    This file contains the Application Program Interface (API) definition  for 
    the Class B Safety Software Library for PIC® MCUs and dsPIC® DSCs.
    
  Description:
    The Class B Safety Software Library includes several
    APIs, which are intended to maximize application
    reliability through Fault detection. These APIs help meet
    the IEC 60730 standard compliance.
    These routines can be directly integrated with the end
    user’s application to test and verify the critical functionalities
    of a controller without affecting the end user’s application.
    
*******************************************************************************/
/*******************************************************************************
FileName:       SSL_ClockTest.h
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

#ifndef __SSL_CLOCK_TEST_H__
#define __SSL_CLOCK_TEST_H__


// *****************************************************************************
// CPU Clock Test Definitions

/* CPU Clock Test Secondary Oscillator (SOSC) reference frequency 

  Summary:
    Defines the reference frequency that is applied to the SOSC input.

  Description:
    This definition is used for internal calculation in the
    SSL_32bitsFamily_CLOCKtest() function.
    Normally this input is driven by a 32.768 KHz xtal.
    You can adjust this value if the need occurs (temperature drifts, etc).
*/

#define     CLOCK_TEST_SOSC_FREQ        32768


// *****************************************************************************
// CPU Clock Test Result

/* CPU Clock Test Success 

  Summary:
    Indicate that the CPU Clock Test has succeeded.

  Description:
    This definition is used as a return value from the
    SSL_32bitsFamily_CLOCKtest() function.
    It indicates that the test was sucessfully passed.
*/

#define CLOCK_TEST_PASS    1


/* CPU Clock Test Fail 

  Summary:
    Indicate that the CPU Clock Test has failed beacuse
    the clock value was found to be greater than the
    specified high clock limit.

  Description:
    This definition is used as a return value from the
    SSL_32bitsFamily_CLOCKtest() function.
    It indicates that the test was not sucessfully passed.
*/

#define CLOCK_TEST_FAIL_HI    -1

/* CPU Clock Test Fail 

  Summary:
    Indicate that the CPU Clock Test has failed beacuse
    the clock value was found to be less than the
    specified low clock limit.

  Description:
    This definition is used as a return value from the
    SSL_32bitsFamily_CLOCKtest() function.
    It indicates that the test was not sucessfully passed.
*/

#define CLOCK_TEST_FAIL_LOW    -2


/*******************************************************************************
  Function:
    int SSL_32bitsFamily_CLOCKtest(unsigned int sysClk, int nMs, int hiClkErr, int loClkErr )

  Summary:
    The CPU Clock test is one of the tests that check
    the reliability of the system clock.
    It implements the independent time slot monitoring
    H.2.18.10.4 as defined by the IEC 60730 standard.
    

  Description:
    The CPU Clock test verifies that the system clock is within specified limits.
    The secondary oscillator (SOSC) is used as the reference clock.
    The CPU Core Timer that runs on the CPU system clock is monitored.

    
    The test performs the following major steps:
        1. The LP SOSC is used as the independent clock source/reference clock
           source connected to hardware Timer1.
        2. The CPU Core Timer monitored in this measurement is incremented every other CPU system clock.
           Usually the system runs on the Primary oscillator with PLL as the clock source to the CPU.
           However, any clock source except the SOSC itself which is used as a reference is valid for this test.
        3. Timer1 is configured to time out after the specified interval of time elapsed (e.g. 10 ms).
        4. The content of the Core Timer is saved at the beginning of the measurement, once the Timer 1
           is started.
        5. When the hardware Timer1 times out another reading of the Core Timer is taken and the difference
           is made with the start value.
           This difference value represents the number of CPU clock cycles counted by the Core Timer during
           the SOSC period of time.
        6. If this value crosses the defined boundary limits the function returns an appropriate
           error value, specifying which exactly limit (upper/lower) was violated.
           
                                                                             
  Precondition:
    None.

  Parameters:
    sysClk              - the current system running frequency, Hz

    nMs                 - number of milliseconds to be used for the CPU Clock monitoring.
                          1 <= nMs <= 1000 

    hiClkErr            - the upper error limit for the system clock, Hz.
                          A monitored value greater than (sysClk+hiClkErr) will trigger the return
                          of the CLOCK_TEST_FAIL_HI error code.
                          
    loClkErr            - the lower error limit for the system clock, Hz.
                          A monitored value less than (sysClk-loClkErr)  will trigger the return
                          of the CLOCK_TEST_FAIL_LOW error code.
    
  Returns:
    Result identifying the pass/fail status of the test:
      CLOCK_TEST_PASS       - The test passed. The monitored CPU clock is within the requested limits.

      CLOCK_TEST_FAIL_HI    - The test failed. The monitored CPU clock is greater than the specified upper limit.

      CLOCK_TEST_FAIL_LOW   - The test failed. The monitored CPU clock is less than the specified lower limit.

      
  Example:
    <code>
    int testRes=SSL_32bitsFamily_CLOCKtest(80000000, 100, 80000, 100000);
    if(testRes==CLOCK_TEST_PASS)
    {
        // process test success
    }
    else if(testRes==CLOCK_TEST_FAIL_HI)
    {
        // process CPU clock high failure
    }
    else
    {
        // process CPU clock low failure
    }
    </code>

  Remarks:

    The test uses the hardware Timer1. It initializes the timer as needed and,
    after the test is done, shuts off the timer.
    The previous state of Timer1 is not preserved/restored.

    The test assumes that the Core Timer is enabled and linearly counting up as it
    should do during normal system operation.
    If your code specifically disables the Core Timer, it should enable it before
    this test is called.
    If the value in the Core Timer is updated/changed as part of an ISR, this ISR should
    be disabled.

    The interrupts should be disabled when executing this test as the time spent in
    ISR's affects the accurate timing of this routine.
    
    The SOSC is used as a reference.
    Use the CLOCK_TEST_SOSC_FREQ for adjustments to this value, if needed.
    
    The value of the CPU clock monitoring time, nMs, is limited because of the use
    of the hardware Timer1 which is a 16 bit timer.
    Therefore, the value loaded into this Timer1 register should not exceed 2^16-1.
    
    Refer to the AN1229 for details regarding the SSL_32bitsFamily_CLOCKtest()
    and the Class B Software Library.
  *****************************************************************************/

int SSL_32bitsFamily_CLOCKtest(unsigned int sysClk, int nMs, int hiClkErr, int loClkErr );



#endif // __SSL_CLOCK_TEST_H__

