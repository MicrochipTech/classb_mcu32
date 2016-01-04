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
FileName:       SSL_CpuRegisterTest.h
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


#ifndef __SSL_CPUREGISTERTEST_H__
#define __SSL_CPUREGISTERTEST_H__


// *****************************************************************************
// CPU Register Test Result

/* CPU Register Test Success 

  Summary:
    Indicate that the CPU Register Test has succeeded.

  Description:
    This definition is used as a return value from the
    SSL_32bitsFamily_CPU_RegisterTest() function.
    It indicates that the test was sucessfully passed.
*/

#define CPU_REGISTER_TEST_PASS    1


/* CPU Register Test Fail 

  Summary:
    Indicate that the CPU Register Test has failed.

  Description:
    This definition is used as a return value from the
    SSL_32bitsFamily_CPU_RegisterTest() function.
    It indicates that the test was not sucessfully passed.
*/

#define CPU_REGISTER_TEST_FAIL    0





/*******************************************************************************
  Function:
    int SSL_32bitsFamily_CPU_RegisterTest ( void )

  Summary:
    The CPU Register test implements the functional test
    H.2.16.5 as defined by the IEC 60730 standard.
    

  Description:
    This routine detects stuck-at Faults in the CPU registers.
    This ensures that the bits in the registers are not stuck at
    a value ‘0’ or ‘1’.

  Precondition:
    None.

  Parameters:
    None.
    
  Returns:
    Result identifying the pass/fail status of the test:
      CPU_REGISTER_TEST_PASS    - The test passed. CPU registers have not been detected to have stuck bits.

      CPU_REGISTER_TEST_FAIL    - The test failed. Some CPU register(s) has been detected to have stuck bits. 

  Example:
    <code>
    int testRes=SSL_32bitsFamily_CPU_RegisterTest();
    if(testRes==CPU_REGISTER_TEST_PASS)
    {
        // process test success
    }
    else
    {
        // process tests failure
    }
    </code>

  Remarks:
    This is a non-destructive test.
    
    Interrupts should be disabled when calling this test function.

    Refer to the AN1229 for details regarding the SSL_32bitsFamily_CPU_RegisterTest()
    and the Class B Software Library.
  *****************************************************************************/

int SSL_32bitsFamily_CPU_RegisterTest(void);

#endif  // __SSL_CPUREGISTERTEST_H__

