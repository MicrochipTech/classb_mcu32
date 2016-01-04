/*******************************************************************************
  Class B Library Implementation File

  Summary:
    This file contains the implementation for 
    the Class B Safety Software Library CPU Clock test
    for PIC32MX MCUs.
    
*******************************************************************************/
/*******************************************************************************
FileName:       SSL_ClockTest_LineFreq.c
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

#include "ClassB/SSL_ClockTest_LineFreq.h"

#include <plib.h>


/*******************************************************************************
  Function:
    int SSL_32bitsFamily_CLOCKtest_LineFreq(unsigned int sysClk, unsigned int lineRefClk, int hiClkErr, int loClkErr)

  Summary:
    The CPU Clock Line Test is one of the tests that check
    the reliability of the system clock.
    It implements the independent time slot monitoring
    H.2.18.10.4 as defined by the IEC 60730 standard.
    

  Description:
    The CPU Clock Line Test verifies that the system clock is within specified limits.
    An external frequency applied on Input Capture 1 pin (IC1) is used as the reference clock.
    The hardware Timer2 that runs on the system Peripheral Bus (PB) clock is used to
    monitor the CPU clock and Peripheral Bus divider.

    
    The test performs the following major steps:
        1. The IC1 input is used as the independent clock source/reference clock
           source to capture the hardware Timer2.
           An external reference frequency, usually the line frequency, has to be applied
           to the IC1 input pin.

        2. The Input Capture 1 is configured as follows:
            - Timer2 is selected as IC1 time base
            - Capture is performed on every rising edge
            - Capture done event is generated on every second capture.

        3. The hardware Timer2 pre-scaler is calculated (based on the input reference
           frequency and the current PB frequency) as being the smallest divider possible
           such that the 16 bit Timer2 does not overflow within a period time of the 
           input reference signal.
           This way, the best resolution is achieved for the given conditions.
           If no valid pre-scaler value can be obtained an error value is returned.
           
        4. The IC1 performs the capture on every rising edge of the input reference frequency.
           For period measurement, the capture done event is generated after the IC1 module
           takes two time stamps i.e. after every period of the input reference (20 ms if
           reference frequency is 50 Hz, 16.66ms if the reference frequency is 60 Hz).
       
        5. Once the capture done event is signalled, the 2 Timer2 captured readings are extracted
           and the number of elapsed PB clocks is calculated as being the difference between
           the two readings.
           If this value crosses the defined boundary limits the function returns an appropriate
           error value, specifying which exactly limit (upper/lower) was violated.
           

       Calculation example 1:
           System Clock     = 80 MHz
           PB Clock         = 80 MHz (PB divider =1:1)
           Input Reference  = 50 Hz

           T2 Min Divider = floor(PBclk/(65536*RefClk))+1 = 25
           Actual T2 Divider = 32.
           The number of cycles counted in the Reference clock period is  = (80,000,000/32)/50 = 50,000.

           
       Calculation example 2:
           System Clock     = 80 MHz
           PB Clock         = 10 MHz (PB divider =1:8)
           Input Reference  = 60 Hz

           T2 Min Divider = floor(PBclk/(65536*RefClk))+1 = 3
           Actual T2 Divider = 4.
           The number of cycles counted in the Reference clock period is  = (10,000,000/4)/60 = 41,666.


  Precondition:
    None.

  Parameters:
    sysClk              - the current system running frequency, Hz

    lineRefClk          - the frequency of the reference applied to the IC1 input pin, Hz.
                          Usual values are 50/60 Hz 

    hiClkErr            - the upper error limit for the system clock, Hz.
                          A monitored value greater than (sysClk+hiClkErr) will trigger the return
                          of the CLOCK_TEST_FAIL_HI error code.
                          
    loClkErr            - the lower error limit for the system clock, Hz.
                          A monitored value less than (sysClk-loClkErr)  will trigger the return
                          of the CLOCK_TEST_FAIL_LOW error code.
    
  Returns:
    Result identifying the pass/fail status of the test:
      CLOCK_TEST_PASS         - The test passed. The monitored CPU clock is within the requested limits.

      CLOCK_TEST_FAIL_HI      - The test failed. The monitored CPU clock is greater than the specified upper limit.

      CLOCK_TEST_FAIL_LOW     - The test failed. The monitored CPU clock is less than the specified lower limit.

      CLOCK_TEST_FAIL_LOW_REF - The test failed. The frequency of the provided reference was too low
                                and could not be used. 
         
      
  Example:
    <code>
    int testRes=SSL_32bitsFamily_CLOCKtest_LineFreq(80000000, 60, 80000, 100000);
    if(testRes==CLOCK_TEST_PASS)
    {
        // process test success
    }
    else if(testRes==CLOCK_TEST_FAIL_HI)
    {
        // process CPU clock high failure
    }
    else if(testRes==CLOCK_TEST_FAIL_LOW)
    {
        // process CPU clock low failure
    }
    else
    {
        // process reference failure
    }
    </code>

  Remarks:


    The test uses the hardware Input Capture 1 module.
    It initializes the module as needed and, after the test is done,
    it shuts it off.
    The previous state of IC1 is not preserved/restored.
  
    The test uses the hardware Timer2. It initializes the timer as needed and,
    after the test is done, shuts off the timer.
    The previous state of Timer1 is not preserved/restored.

    The value of the PB frequency which is used as input by the Timer2 is derived from the
    system CPU clock by dividing it with the PB divider.
    The test does not change the value of the PB divider, it uses the current value.

    The interrupts should be disabled when executing this test as the time spent in
    ISR's affects the accurate timing of this routine.
    
    The frequency of signal used as a reference on IC1 input should normally be the line frequency.
    However, any frequency can be used as long as a a valid Timer2 divider can be obtained
    (see the example calculation below for the 16 bit Timer2).
    If the reference frequency is too low, a valid divider for the Timer 2 won't be possible.
    You can go to a greater PB divider in this case.
    If the selected reference frequency is too high the number of Timer2 captured counts
    will be too small and the measurement won't have enough resolution. 
        
    Refer to the AN1229 for details regarding the SSL_32bitsFamily_CLOCKtest_LineFreq()
    and the Class B Software Library.
  *****************************************************************************/

int SSL_32bitsFamily_CLOCKtest_LineFreq(unsigned int sysClk, unsigned int lineRefClk, int hiClkErr, int loClkErr )
{

    unsigned int    pbClk, t2Div, t2DivPwr;
    unsigned int    refCntHiVal, refCntLoVal;
    unsigned short  t1, t2, currCount;
    int             testResult;
    
    // T2 input clock
    pbClk=sysClk>>mOSCGetPBDIV();
     
    // calculate the T2 minimum divider value
    t2Div=pbClk/(65536*lineRefClk)+1;

    // the max divider supported is 256
    if(t2Div>256)
    {   
        return CLOCK_TEST_FAIL_LOW_REF; // we need a too great divider, the reference is too low
    }

    t2DivPwr=0;
	while(t2Div>(1<<t2DivPwr))
	{
		t2DivPwr++;
	}

    // adjust   
    if(t2DivPwr>6)
    {
        t2DivPwr=7;   // 128 divider not supported on T2
        t2Div=256;
    }
    else
    {
        t2Div=1<<t2DivPwr;
    }

    // open the T2 channel
    OpenTimer2(T2_ON | (t2DivPwr<<_T2CON_TCKPS_POSITION), -1);
    
    // calculate the imposed limits
    // Sys Clk ->PB Divider -> T2 divider -> in reg clock time    
    refCntHiVal=(((sysClk+hiClkErr)>>mOSCGetPBDIV())/lineRefClk)/t2Div;
    refCntLoVal=(((sysClk-loClkErr)>>mOSCGetPBDIV())/lineRefClk)/t2Div;


    // init the input capture
    OpenCapture1( IC_ON | IC_TIMER2_SRC | IC_INT_2CAPTURE | IC_EVERY_RISE_EDGE);
    while(mIC1CaptureReady())
    {
        mIC1ReadCapture();        
    }

  // sync first
   mIC1ClearIntFlag();
   while( !mIC1GetIntFlag());  
   mIC1ReadCapture();
   mIC1ReadCapture();
 
  // wait for two consecutive captures -> interrupt flag set
   mIC1ClearIntFlag();
   while( !mIC1GetIntFlag());  

   // read the two captured values from FIFO
   t1 = mIC1ReadCapture();
   t2 = mIC1ReadCapture();
   
   CloseCapture1();
   CloseTimer2();
     
    // check the captured counter value       
    currCount = t2-t1; 
    if((unsigned int)currCount > refCntHiVal)
    {
        testResult=CLOCK_TEST_FAIL_HI;
    }
    else if((unsigned int)currCount < refCntLoVal )
    {
        testResult=CLOCK_TEST_FAIL_LOW;
    }
    else
    {
        testResult=CLOCK_TEST_PASS;
    }

    
    return testResult;
   
}

