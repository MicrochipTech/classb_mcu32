/*******************************************************************************
  Class B Library implementation file.

  Summary:
    This file contains the implementation for the Program Counter test for 
    the Class B Safety Software Library for PIC32MX.
        
*******************************************************************************/
/*******************************************************************************
FileName:       SSL_PcTest.c
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

#include "ClassB/SSL_PcTest.h"

/*******************************************************************************
  Function:
    int SSL_32bitsFamily_PCtest (void)

  Summary:
    The Program Counter test implements one of the functional tests
    H.2.16.5 as defined by the IEC 60730 standard.
    

  Description:
    The Program Counter test is a functional test of the PC.
    It checks that the PC register is not stuck and it properly holds the address
    of the next instruction to be executed.
    
    The tests performs the following major tasks:
        1. The Program Counter test invokes functions that are located in memory at different addresses.
        2. These functions return a unique value( address of the respective functions).
        3. The returned value is verified using the "SSL_32bitsFamily_PCtest" function.

                                                                             
  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Result identifying the pass/fail status of the test:
        PC_TEST_PASS    - The test passed. The PC register holds the correct address.

        PC_TEST_FAIL    - The test failed. The PC register has been detected to hold an incorrect address. 

  Example:
    <code>
    int testRes=SSL_32bitsFamily_PCtest();
    if(testRes==PC_TEST_PASS)
    {
        // process test success
    }
    else
    {
        // process tests failure
    }
    </code>

  Remarks:

    The test uses 3 different functions:
        SSL_TestPCFunction1()
        SSL_TestPCFunction2()          
        SSL_TestPCFunction3()          
    The ROM location of these functions that are used for PC test at run time
    can be changed by modifying the provided elf32pic32mx.ld file.
    Look for the .SSL_TestPCFunction1 sections and the like.

    Add the elf32pic32mx.ld file to your project.
    
    Refer to the AN1229 for details regarding the SSL_32bitsFamily_PCtest()
    and the Class B Software Library.
  *****************************************************************************/

int SSL_32bitsFamily_PCtest(void)
{
    int     ix;
    int     result;
    void   *returnAddress;
    
    // table storing the functions to branch to
    typedef void* (*pTestFnc)(void);        // pointer to the test function
    static const pTestFnc   testFbcTbl[4]=
    {
        SSL_TestPCFunction1,        // address bits 0x00
        SSL_TestPCFunction2,        // address bits 0x55
        SSL_TestPCFunction3,        // address bits 0xaa
        SSL_TestPCFunction1         // toggle back to 0
    };
        
    

    result=PC_TEST_PASS;
    
    for(ix=0; ix<sizeof(testFbcTbl)/sizeof(*testFbcTbl); ix++)
    {
        returnAddress=(*testFbcTbl[ix])();
        if(returnAddress!=testFbcTbl[ix])
        {
            result=PC_TEST_FAIL;
            break;
        }
    }

    return result;
    
}



/*******************************************************************************
  Function:
    void* SSL_TestPCFunction1(void)
    void* SSL_TestPCFunction2(void)
    void* SSL_TestPCFunction3(void)

  Summary:
    Helpers for the Program Counter functional test.
    

  Description:
    These are helper functions that return their own address.
                                                                             
  Precondition:
    None.

  Parameters:
    None. 

  Returns:
    Each function returns its own address.

  Example:
    See the SSL_32bitsFamily_PCtest() example.

  Remarks:
    The functions are placed in their own sections called
        ".SSL_TestPCFunction1"
        ".SSL_TestPCFunction2"
        ".SSL_TestPCFunction3"

    The sections have to be located at meaningful addresses where the PC needs to jump using the provided
    custom linker script elf32pic32mx.ld.
        
    The provided linker script file is for a part with 512 KB of Flash memory.
    So the valid Flash addresses are 0x9d000_0000 to 0x9d07_ffff;
    The linker script has to be updated for each specific processor variant.
    
    In this example the sections used for the PC test functions are placed at the follwoing adddresses:
        section .SSL_TestPCFunction1 at the address 0x9d000000 (all the Flash address bits are 0).
        section .SSL_TestPCFunction2 at the address 0x9d02aaa8 (all the uneven Flash address bits are toggled to 1).
        section .SSL_TestPCFunction3 at the address 0x9d055554 (all the even Flash address bits are toggled to 1).
        
    If the elf32pic32mx.ld script is not added to the project
    the linker will place the test functions at default values.
        
    Other similar test functions could be added using the exact same model.
  *****************************************************************************/
void* __attribute__((__section__(".SSL_TestPCFunction1"))) SSL_TestPCFunction1(void) 
{
	return  SSL_TestPCFunction1;
}

void* __attribute__((__section__(".SSL_TestPCFunction2"))) SSL_TestPCFunction2(void) 
{
	return SSL_TestPCFunction2;
}

void* __attribute__((__section__(".SSL_TestPCFunction3"))) SSL_TestPCFunction3(void) 
{
	return SSL_TestPCFunction3;
}

