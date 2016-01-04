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
FileName:       SSL_MarchC.h
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

#ifndef __SSL_MARCHC_H__
#define __SSL_MARCHC_H__


// *****************************************************************************
// RAM March C Test Result

/* RAM  March C Test Success 

  Summary:
    Indicate that the RAM  March C Test has succeeded.

  Description:
    This definition is used as a return value from the
    SSL_32bitsFamily_RAMtest_MarchC() function.
    It indicates that the test was sucessfully passed.
*/

#define MARCHC_TEST_PASS    1


/* RAM  March C Test Fail 

  Summary:
    Indicate that the RAM  March C Test has failed.

  Description:
    This definition is used as a return value from the
    SSL_32bitsFamily_RAMtest_MarchC() function.
    It indicates that the test was not sucessfully passed.
*/

#define MARCHC_TEST_FAIL    0


/* Stack March C Test Fail 

  Summary:
    Indicate that the Stack March C Test has failed.

  Description:
    This definition is used as a return value from the
    SSL_32bitsFamily_RAM_STACKtest_MarchC() function.
    It indicates that there was not enough
    space in the RAM area to save the Stack area.
    The Stack March C test was not performed.
*/

#define MARCHC_TEST_FAIL_SIZE   -1


/* Stack March C Test Fail 

  Summary:
    Indicate that the Stack March C Test has failed.

  Description:
    This definition is used as a return value from the
    SSL_32bitsFamily_RAM_STACKtest_MarchC() function.
    It indicates that the Stack area requested to be tested
    does not completely contain the current hardware stack area
    (as pointed by the SP register).
    The Stack March C test was not performed.
*/

#define MARCHC_TEST_FAIL_STACK  -2

/*******************************************************************************
  Function:
    int SSL_32bitsFamily_RAMtest_MarchC (int* ramStartAddress, int ramSize)

  Summary:
    The RAM March C test is one of the Variable Memory tests
    that implements the Periodic Static Memory test
    H.2.19.6 as defined by the IEC 60730 standard.
    

  Description:
    This test is a complete and non redundant test capable of detecting
    stuck-at, addressing, transition and coupling faults.
    This test is of complexity 11n( Where n is the number of bits tested). 
    The test uses word (32-bit) accesses.
    The address must be properly word aligned and the length of the
    area to be tested must be an integral multiple of the data width access.
                                                                             
  Precondition:
    None.

  Parameters:
    ramStartAddress     - start Address from which the March C test is to be performed
                          Must be properly 32 bit aligned.

    ramSize             - number of consecutive byte locations for which the test is to be performed
                          The size must be a number multiple of 4.
    
  Returns:
    Result identifying the pass/fail status of the test:
      MARCHC_TEST_PASS    - The test passed. RAM area tested has not been detected to have faults. 

      MARCHC_TEST_FAIL    - The test failed. Some RAM area location has been detected to have faults. 

  Example:
    <code>
    int testRes=SSL_32bitsFamily_RAMtest_MarchC(startAddress, size);
    if(testRes==MARCHC_TEST_PASS)
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
    
    At least 100 bytes should be available for stack for executing the March C test.
    The tested RAM area must not overlap the stack.
    
    Other statically allocated resources,  such as the MPLAB ICD/Real ICE
    allocated RAM buffers should be excluded from this test.    
    
    The Start Address from which the March C test is to be performed is
    PIC32 variant and application dependent. It is a run-time parameter.
    
    The routine accesses one 4 byte RAM word at a time.        
    
    Refer to the AN1229 for details regarding the SSL_32bitsFamily_RAMtest_MarchC()
    and the Class B Software Library.
  *****************************************************************************/

int SSL_32bitsFamily_RAMtest_MarchC(int* ramStartAddress, int ramSize);


/*******************************************************************************
  Function:
    int SSL_32bitsFamily_RAMtest_MarchC_Minus (int* ramStartAddress, int ramSize)

  Summary:
    The RAM March C Minus test is one of the Variable Memory tests
    that implements the Periodic Static Memory test
    H.2.19.6 as defined by the IEC 60730 standard.
    

  Description:
    This test is a complete and non redundant test capable of detecting
    stuck-at, addressing, transition and coupling faults.
    This test is of complexity 10n( Where n is the number of bits tested). 
    The test uses word (32-bit) accesses.
    The address must be properly word aligned and the length of the
    area to be tested must be an integral multiple of the data width access.
                                                                             
  Precondition:
    None.

  Parameters:
    ramStartAddress     - start Address from which the March C Minus test is to be performed
                          Must be properly 32 bit aligned.

    ramSize             - number of consecutive byte locations for which the test is to be performed
                          The size must be a number multiple of 4.
    
  Returns:
    Result identifying the pass/fail status of the test:
      MARCHC_TEST_PASS    - The test passed. RAM area tested has not been detected to have faults. 

      MARCHC_TEST_FAIL    - The test failed. Some RAM area location has been detected to have faults. 

  Example:
    <code>
    int testRes=SSL_32bitsFamily_RAMtest_MarchC_Minus(startAddress, size);
    if(testRes==MARCHC_TEST_PASS)
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
    
    At least 100 bytes should be available for stack for executing the March C test.
    The tested RAM area must not overlap the stack.
    
    Other statically allocated resources,  such as the MPLAB ICD/Real ICE
    allocated RAM buffers should be excluded from this test.    
    
    The Start Address from which the March C Minus test is to be performed is
    PIC32 variant and application dependent. It is a run-time parameter.
    
    The routine accesses one 4 byte RAM word at a time.        
    
    Refer to the AN1229 for details regarding the SSL_32bitsFamily_RAMtest_MarchC_Minus()
    and the Class B Software Library.
  *****************************************************************************/

int SSL_32bitsFamily_RAMtest_MarchC_Minus(int* ramStartAddress, int ramSize);


/*******************************************************************************
  Function:
    int SSL_32bitsFamily_RAM_STACKtest_MarchC (int* ramStartAddress, int ramSize, int* stackTopAddress, int stackSize)

  Summary:
    The RAM March C test is one of the Variable Memory tests
    that implements the Periodic Static Memory test
    H.2.19.6 as defined by the IEC 60730 standard.
    

  Description:
    This test is a complete and non redundant test capable of detecting
    stuck-at, addressing, transition and coupling faults.
    This test is of complexity 11n( Where n is the number of bits tested). 
    The test uses word (32-bit) accesses.
    The addresses must be properly word aligned and the lengths of the
    areas to be tested must be an integral multiple of the data width access.
                                                                             
  Precondition:
    None.

  Parameters:
    ramStartAddress     - start Address of RAM area for which the March C test is to be performed
                          Has to NOT overlap the Stack area!
                          Must be properly 32 bit aligned.

    ramSize             - number of consecutive byte locations for which the test is to be performed
                          The size must be a number multiple of 4.
                          The size of the RAM area tested has to be >= 128 bytes.
                          
    stackTopAddress     - address of the top of the Stack area for which the March C test is to be performed
                          Note that the stack is supposed to grow downwards!
                          Has to NOT overlap the RAM area!
                          Must be properly 32 bit aligned.

    stackSize            - number of consecutive byte locations in the Stack area for which the test is to be performed
                          The size must be a number multiple of 4.
                          It has to be < ramSize
    
  Returns:
    Result identifying the pass/fail status of the test:
      MARCHC_TEST_PASS          - The test passed. RAM and Stack area tested have not been detected to have faults. 

      MARCHC_TEST_FAIL          - The test failed. Either some RAM or Stack area location has been detected to have faults. 

      MARCHC_TEST_FAIL_SIZE     - The test failed. There was not enough space in the RAM area to save the Stack area

      MARCHC_TEST_FAIL_STACK    - The test failed. The requested Stack area does not actually contain the current hardware SP register.

      
      
  Example:
    <code>
    int testRes=SSL_32bitsFamily_RAM_STACKtest_MarchC(startAddress, size, stackTopAddress, stackSize);
    if(testRes==MARCHC_TEST_PASS)
    {
        // process test success
    }
    else
    {
        // process tests failure
    }
    </code>

  Remarks:

    This function is just a helper to March C test both a regular RAM area and a Stack area.
    First the RAM area is tested using the standard March C test.
    If the test succeeded, the requested Stack area is saved/copied in the RAM area
    that has just been tested and then the March C test is run over the Stack area
    as if it were a regular RAM area.
    The saved Stack area is restored and the result of the test is returned to the user.

    The RAM and Stack areas have to NOT overlap!
    The Stack grows downwards so the tested area is:
    [stackTopAddress-stackSize, stackTopAddress]
    Also the size of the Stack area to be tested has to be less than the size of the RAM area.
 
    The processor SP register is changed to the point to the RAM area while the Stack area is tested.
    Since running the MARC C test requires at least 128 bytes of stack, the RAM area size should be at least
    128 bytes long.
    Once the Stack area is tested, the SP register is restored.
    
    This is a destructive memory test.
    Either exclude from this test RAM areas that have to be preserved
    or save/restore the memory area before/after running the test
    or run the test at system startup before the memory and the
    run time library is initialized (stack needs to be initialized though).
    
    At least 128 bytes should be available for stack for executing the March C test.
    The tested RAM area must not overlap the stack.
    
    Other statically allocated resources,  such as the MPLAB ICD/Real ICE
    allocated RAM buffers should be excluded from this test.    
    
    The Start Address from which the March C test is to be performed is
    PIC32 variant and application dependent. It is a run-time parameter.
    
    The routine accesses one 4 byte RAM word at a time.        
    
    Refer to the AN1229 for details regarding the SSL_32bitsFamily_RAM_STACKtest_MarchC()
    and the Class B Software Library.
  *****************************************************************************/

int SSL_32bitsFamily_RAM_STACKtest_MarchC(int* ramStartAddress, int ramSize, int* stackTopAddress, int stackSize);


    
#endif  // __SSL_MARCHC_H__

