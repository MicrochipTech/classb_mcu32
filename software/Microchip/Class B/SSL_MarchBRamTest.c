/*******************************************************************************
  Class B Library implementation file

  Summary:
    This file contains the implementation for the Class B Safety
    Software Library March B RAM test for PIC32MX MCUs.
    
*******************************************************************************/
/*******************************************************************************
FileName:       SSL_MarchBRamTest.c
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

#include "ClassB/SSL_MarchB.h"


// definitions

#define MARCHB_BIT_WIDTH        32      //  access data width


// prototypes
// 
static int      Rd0Wr1Rd1Wr0Rd0Wr1( volatile int* ptr );
static int      Rd1Wr0Wr1( volatile int* ptr );
static int      Rd1Wr0Wr1Wr0( volatile int* ptr );
static int      Rd0Wr1Wr0( volatile int* ptr );



/*******************************************************************************
  Function:
    int SSL_32bitsFamily_RAMtest_MarchB (int* ramStartAddress, int ramSize)

  Summary:
    The RAM March B test is one of the Variable Memory tests
    that implements the Periodic Static Memory test
    H.2.19.6 as defined by the IEC 60730 standard.
    

  Description:
    This test is a complete and non redundant test capable of detecting
    stuck-at, linked idempotent coupling or Inversion Coupling faults.
    This test is of complexity 17n( Where n is the number of bits tested). 
    The test uses word (32-bit) accesses.
    The address must be properly word aligned and the length of the
    area to be tested must be an integral multiple of the data width access.
                                                                             
  Precondition:
    None.

  Parameters:
    ramStartAddress     - start Address from which the March B test is to be performed
                          Must be properly 32 bit aligned.

    ramSize             - number of consecutive byte locations for which the test is to be performed
                          The size must be a number multiple of 4.
    
  Returns:
    Result identifying the pass/fail status of the test:
      MARCHB_TEST_PASS    - The test passed. RAM area tested has not been detected to have faults. 

      MARCHB_TEST_FAIL    - The test failed. Some RAM area location has been detected to have faults. 

  Example:
    <code>
    int testRes=SSL_32bitsFamily_RAMtest_MarchB(startAddress, size);
    if(testRes==MARCHB_TEST_PASS)
    {
        // process test success
    }
    else
    {
        // process tests failure
    }
    </code>

  Remarks:
    This is a destructive memory test.
    Either exclude from this test RAM areas that have to be preserved
    or save/restore the memory area before/after running the test
    or run the test at system startup before the memory and the
    run time library is initialized (stack needs to be initialized though).
    
    At least 100 bytes should be available for stack for executing the March B test.
    The tested RAM area must not overlap the stack.
    
    Other statically allocated resources,  such as the MPLAB ICD/Real ICE
    allocated RAM buffers should be excluded from this test.    
    
    The Start Address from which the March B test is to be performed is
    PIC32 variant and application dependent. It is a run-time parameter.
    
    The routine accesses one 4 byte RAM word at a time.        
    
    Refer to the AN1229 for details regarding the SSL_32bitsFamily_RAMtest_MarchB()
    and the Class B Software Library.
  *****************************************************************************/

int  SSL_32bitsFamily_RAMtest_MarchB(int* ramStartAddress, int ramSize)
{
    int testResult;
    int *ramEndAddress;     
 	int  *ptr;   	

    ramEndAddress=ramStartAddress + (ramSize/4);    // ramSize should be in bytes
  
	// erase all sram
    for(ptr=ramStartAddress; ptr<ramEndAddress; ptr++)
    {
        *ptr = 0;       //write 0
    }

    testResult=MARCHB_TEST_PASS;

    for(ptr=ramStartAddress; ptr<ramEndAddress && testResult==MARCHB_TEST_PASS; ptr++)
    {
        testResult = Rd0Wr1Rd1Wr0Rd0Wr1(ptr); 
    }

    for(ptr=ramStartAddress; ptr<ramEndAddress && testResult==MARCHB_TEST_PASS; ptr++)
    {  
        testResult = Rd1Wr0Wr1(ptr);
    } 

    for (ptr=ramEndAddress-1; ptr>=ramStartAddress && testResult==MARCHB_TEST_PASS; ptr--) 
    {  
        testResult = Rd1Wr0Wr1Wr0(ptr);
    }

    for (ptr=ramEndAddress-1; ptr>=ramStartAddress && testResult==MARCHB_TEST_PASS; ptr--) 
    {
        testResult = Rd0Wr1Wr0(ptr);  
    }

    return testResult;
}

  /**************************************************************
  * Description:
  *     This function does the following :
  *     
  *     1> Tests bitwise if bit is zero and replace with one .
  *     
  *     2> Tests bitwise if bit is one and replace with zero.
  *     
  *     3> Tests bitwise if bit is zero and replace with one.
  * Input:
  *     ptr :  Address location of the the bits to be tested.
  *
  * Return Values:
  *     MARCHB_TEST_FAIL : test failed
  *     MARCHB_TEST_PASS : test passed
  *                                                            
  **************************************************************/

static int Rd0Wr1Rd1Wr0Rd0Wr1( volatile int * ptr)
{

    int tempValue;  
    int loopCounter;

    for (loopCounter=0; loopCounter<MARCHB_BIT_WIDTH; loopCounter++)
    {

        tempValue =  (((*ptr) >> loopCounter) & 1);   //Read 0

        if( tempValue != 0)
        { 
            return MARCHB_TEST_FAIL;
        }

        tempValue = *ptr | (1<<loopCounter);             // write 1
        *ptr= tempValue;                              

        tempValue =(((*ptr)>>loopCounter) & 1);       // read 1 

        if (tempValue!= 1) 
        {
            return MARCHB_TEST_FAIL;
        }

        tempValue =  *ptr  & ~(1<<loopCounter);         // write 0
        *ptr= tempValue;                                 

        tempValue =(((*ptr)>>loopCounter) & 1);      // read 0 

        if (tempValue!= 0) 
        {
            return MARCHB_TEST_FAIL;
        } 

        *ptr=(*ptr | (1<<loopCounter));                  // write 1

    } 
    return MARCHB_TEST_PASS; 

}

  /******************************************************************
  * Description:
  *     This function does the following:
  *     
  *     1> Tests bitwise if a bit is one and replace it with Zero.
  *     
  *     2> Write One Bitwise.
  * Input:
  *     ptr :  Address location of the the bits to be tested.
  *
  * Return Values:
  *     MARCHB_TEST_FAIL : test failed
  *     MARCHB_TEST_PASS : test passed
  *                                                                
  ******************************************************************/
static int Rd1Wr0Wr1( volatile int * ptr)
{
    int tempValue;  
    int loopCounter;

    for (loopCounter=0; loopCounter<MARCHB_BIT_WIDTH; loopCounter++)
    {
        tempValue =(((*ptr)>>loopCounter) & 1);       // read 1 
        if ( tempValue!= 1) 
        {
            return MARCHB_TEST_FAIL;
        }

        tempValue =  *ptr  & ~(1<<loopCounter);
        *ptr=  tempValue;                                // write 0

        *ptr=(*ptr | (1<<loopCounter));                  // write 1

    }

    return MARCHB_TEST_PASS; 
}

  /******************************************************************
  * Description:
  *     This function does the following:
  *     
  *     1> Tests bitwise if a bit is one and replace it with Zero.
  *     
  *     2> write one Bitwise.
  *     
  *     3> write Zero Bitwise.
  * Input:
  *     ptr :  Address location of the the bits to be tested.
  * Return Values:
  *     MARCHB_TEST_FAIL : test failed
  *     MARCHB_TEST_PASS : test passed
  *                                                                
  ******************************************************************/
 
static int Rd1Wr0Wr1Wr0( volatile int * ptr)
{

    int tempValue;  
    int loopCounter;

    for (loopCounter=MARCHB_BIT_WIDTH-1; loopCounter>=0; loopCounter--)
    {

        tempValue =(((*ptr)>>loopCounter) & 1);     // read 1 
        if (tempValue!= 1) 
        {
            return MARCHB_TEST_FAIL;
        }

        tempValue =  *ptr & ~(1<<loopCounter);
        *ptr= tempValue;                               // write 0

        *ptr=(*ptr | (1<<loopCounter));                // write 1

        tempValue =  *ptr & ~(1<<loopCounter);
        *ptr= tempValue;                               // write 0
    }

    return MARCHB_TEST_PASS; 
}
  /******************************************************************
  * Description:
  *     This function does the following:
  *     
  *     1> Tests bitwise if a bit is zero and replace it with one.
  *     
  *     2> Write Zero.
  * Input:
  *     ptr :  Address location of the the bits to be tested.
  * Return Values:
  *     MARCHB_TEST_FAIL : test failed
  *     MARCHB_TEST_PASS : test passed
  *                                                                
  ******************************************************************/

static int Rd0Wr1Wr0( volatile int * ptr)
{

    int tempValue;  
    int loopCounter;

    for (loopCounter=MARCHB_BIT_WIDTH-1; loopCounter>=0; loopCounter--)
    {
        tempValue =(((*ptr)>>loopCounter) & 1);     // read 0 
        if (tempValue!= 0) 
        {
            return MARCHB_TEST_FAIL;
        } 

        *ptr=(*ptr | (1<<loopCounter));                // write 1

        tempValue =  *ptr & ~(1<<loopCounter);
        *ptr= tempValue;                               // write 0

    }

    return MARCHB_TEST_PASS; 
}


