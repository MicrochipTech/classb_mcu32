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
FileName:       SSL_CBram.h
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

#ifndef __SSL_CBRAM_H__
#define __SSL_CBRAM_H__




// *****************************************************************************
// RAM Checker Board Test Result

/* RAM Checker Board Test Success 

  Summary:
    Indicate that the RAM Checker Board Test has succeeded.

  Description:
    This definition is used as a return value from the
    SSL_32bitsFamily_RAMtest_CheckerBoard() function.
    It indicates that the test was sucessfully passed.
*/

#define CB_TEST_PASS    1


/* RAM Checker Board Test Fail 

  Summary:
    Indicate that the RAM Checker Board Test has failed.

  Description:
    This definition is used as a return value from the
    SSL_32bitsFamily_RAMtest_CheckerBoard() function.
    It indicates that the test was not sucessfully passed.
*/

#define CB_TEST_FAIL    0


/*******************************************************************************
  Function:
    int SSL_32bitsFamily_RAMtest_CheckerBoard (int* ramStartAddress, int ramSize)

  Summary:
    The RAM Checker Board test implements one of the functional tests
    H.2.19.6 as defined by the IEC 60730 standard.
    

  Description:
    This routine detects single bit Faults in the variable memory.
    This ensures that the bits in the tested RAM are not stuck at
    a value ‘0’ or ‘1’.

    The test writes the checkerboard pattern (0x55555555 followed by 0xaaaaaaaa)
    to adjacent memory locations starting at ramStartAddress.
	It performs the following steps:                                           
		1. The content of a 64 bytes memory chunk to be tested is saved in    
		   temporary CPU registers.                                                      
		2. Writes the pattern 0x55555555 followed by 0xaaaaaaaa to adjacent memory locations 
		   filling up the 64 bytes memory chunk.                          
		3. It reads the memory chunk adjacent locations and checks that the read-back values match
           the written pattern.
           If the values match set the success result and go to step 4.
           Else set the error result and go to step 6.
		4. Writes the inverted pattern 0xaaaaaaaa followed by 0x55555555 to adjacent memory locations 
		   filling up the 64 bytes memory chunk.
		5. It reads the memory chunk adjacent locations and checks that the read-back values match
           the written pattern.
           If the values match set the success result.
           Else set the error result.
		6. The content of the tested 64 bytes memory chunk is restored from the
           temporary CPU registers.
        7. If the result shows error the test is done and returns.
        8. The address pointer is incremented to point to the next sequential 64 bytes memory chunk
           and the test is repeated from step 1 until all the number of requested memory locations
           is tested.
                                                                             
  Precondition:
    None.

  Parameters:
    ramStartAddress     - start Address from which the checker Board test is to be performed
                          Must be properly 32 bit aligned.

    ramSize             - number of consecutive byte locations for which the test is to be performed
                          The size must be a number multiple of 64.
    
  Returns:
    Result identifying the pass/fail status of the test:
      CB_TEST_PASS    - The test passed. RAM area tested has not been detected to have stuck bits.

      CB_TEST_FAIL    - The test failed. Some RAM area location has been detected to have stuck bits. 

  Example:
    <code>
    int testRes=SSL_32bitsFamily_RAMtest_CheckerBoard(startAddress, size);
    if(testRes==CB_TEST_PASS)
    {
        // process test success
    }
    else
    {
        // process tests failure
    }
    </code>

  Remarks:
    This is a non-destructive memory test. The content of the tested memory area is saved and restored.
    The test operates in 64 bytes long memory chunks at a time.
    
    At least 32 bytes should be available for stack for executing the RAM Checker Board test.
    The tested RAM area must not overlap the stack.
    
    The Start Address from which the Checker Board test is to be performed is
    PIC32 variant and application dependent. It is a run-time parameter.
    
    The routine accesses one 4 byte RAM word at a time.
    
    Refer to the AN1229 for details regarding the SSL_32bitsFamily_RAMtest_CheckerBoard()
    and the Class B Software Library.
  *****************************************************************************/

int SSL_32bitsFamily_RAMtest_CheckerBoard(int* ramStartAddress, int ramSize);

#endif  // __SSL_CBRAM_H__



