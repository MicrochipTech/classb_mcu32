/*******************************************************************************
  Class B Library implementation file

  Summary:
    This file contains the implementation for the
    the Class B Safety Software Library March C Stack Tests
    for PIC32MX MCUs.
        
*******************************************************************************/
/*******************************************************************************
FileName:       SSL_MarchCStackTest.S
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

/* global defines */

.global SSL_32bitsFamily_RAM_STACKtest_MarchC

/* result codes */
.equ    MARCHC_TEST_PASS, 1
.equ    MARCHC_TEST_FAIL, 0
.equ    MARCHC_TEST_FAIL_SIZE, -1
.equ    MARCHC_TEST_FAIL_STACK, -2



.equ    MARCHC_STACK_DOWN_GUARD, 100    ;# towards 0 stack guard space
                                        ;# it's the stack space taken by the call to SSL_32bitsFamily_RAMtest_MarchC()

.equ    MARCHC_STACK_UP_GUARD, 24       ;# stack save space needed within SSL_32bitsFamily_RAM_STACKtest_MarchC
                                        ;# currently we need a function call with 4 params (16 Bytes)
                                        ;# plus 2 locally saved params (8 Bytes)
                                        ;# totals 24
                                        ;# Note: Adjust accordingly when changes are made! 
.text



/* implementation */

    
.set    noreorder
.set    noat

.text

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
                          Note that the stack is supossed to grow downwards!
                          Has to NOT overlap the RAM area!.
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
    If the test suceeded, the requested Stack area is saved/copied in the RAM area
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
    run time library is initialized (stack needs to be initialzed though).
    
    At least 128 bytes should be available for stack for executing the March C test.
    The tested RAM area must not overlap the stack.
    
    Other statically allocated resources,  such as the MPLAB ICD/Real ICE
    allocated RAM buffers should be excluded from this test.    
    
    The Start Address from which the March C test is to be performed is
    PIC32 variant and application dependent. It is a run-time parameter.
    
    The routine accesses one 4 byte RAM word at a time.        
    
    Refer to the AN1229 for details regarding the SSL_32bitsFamily_RAM_STACKtest_MarchC()
    and the Class B Software Library.

int SSL_32bitsFamily_RAM_STACKtest_MarchC (int* ramStartAddress, int ramSize, int* stackTopAddress, int stackSize);

  *****************************************************************************/
.ent    SSL_32bitsFamily_RAM_STACKtest_MarchC
SSL_32bitsFamily_RAM_STACKtest_MarchC:

    addiu   $sp, $sp, -MARCHC_STACK_UP_GUARD;  # 2 params + function call
                                            ;  # Note: this value has to be properly updated if changes are made!
    ;# save params
    sw      $a0, 24($sp);       # ramStartAddress
    sw      $a1, 28($sp);       # ramSize
    sw      $a2, 32($sp);       # stackTopAddress
    sw      $a3, 36($sp);       # stackSize
    ;# save $ra and $sp
    sw      $ra, 20($sp);
    sw      $sp, 16($sp);

    ;# first test the RAM area
    la      $t0, SSL_32bitsFamily_RAMtest_MarchC;
    jalr    $t0;    # SSL_32bitsFamily_RAMtest_MarchC(ramStartAddress, ramSize);
    nop;
    ori     $t0, $0, MARCHC_TEST_PASS;
    bne     $v0, $t0, StackTestDone;    # RAM test failed
    ;# restore the parameters
    lw      $a0, 24($sp);
    lw      $a1, 28($sp);
    lw      $a2, 32($sp);
    lw      $a3, 36($sp);
    ;# check if we can execute the Stack area test
    sltu    $t0, $a1, $a3;
    bne     $t0, $0, StackTestDone;             # if(ramSize < stackSize)
    addiu   $v0, $0, MARCHC_TEST_FAIL_SIZE;     # testResult=MARCHC_TEST_FAIL_SIZE
    ;#  check that the Stack area completely includes the Used stack space
    addiu   $t0, $sp, -MARCHC_STACK_DOWN_GUARD; # used sp space=[t0, t1]=
    addiu   $t1, $sp, MARCHC_STACK_UP_GUARD;    # = [sp-MARCHC_STACK_DOWN_GUARD, sp+MARCHC_STACK_UP_GUARD]
    subu    $t2, $a2, $a3;                      # t2=stackTopAddress-stackSize
    sltu    $t3, $t0, $t2;                      # if(sp-MARCHC_STACK_DOWN_GUARD < stackTopAddress-stackSize)
    bne     $t3, $0, StackTestDone;             #       testResult=MARCHC_TEST_FAIL_STACK
    addiu   $v0, $0, MARCHC_TEST_FAIL_STACK;    
    sltu    $t3, $a2, $t1;                      # if(stackTop < sp + MARCHC_STACK_UP_GUARD)
    bne     $t3, $0, StackTestDone;             #       testResult=MARCHC_TEST_FAIL_STACK;
    addiu   $v0, $0, MARCHC_TEST_FAIL_STACK;    
    ;# all's good and test can be performed    
    ;# copy downwards the Stack Area to the RAM area
    or      $t0, $0, $a2;   # pSrc=stackTopAddress;
    addu    $t1, $a0, $a1;  # pDst=ramStartAddress+ramSize;
    or      $t2, $0, $a3;   # nBytes=stackSize;
    ;# copy
1:  lw      $t3, -4($t0);
    addiu   $t0, $t0, -4;
    sw      $t3, -4($t1);
    addiu   $t2, $t2, -4;
    bne     $t2, $0, 1b;
    addiu   $t1, $t1, -4;
    ;# copy done; switch the stack
    subu    $t3, $a2, $sp;  # stackOffset=stackTopAddress-sp;
    addu    $t1, $a0, $a1;
    subu    $sp, $t1, $t3;  # sp=ramStartAddress+ramSize-stackOffset;    
    ;# switch done; call the March C RAM test
    subu    $a0, $a2, $a3;
    la      $t0, SSL_32bitsFamily_RAMtest_MarchC;
    jalr    $t0;    # SSL_32bitsFamily_RAMtest_MarchC(stackTopAddress-stackSize, stackSize);
    or      $a1, $0, $a3;
    ;# test done; preserve the return value v0
    ;# restore the stack area
    lw      $a0, 24($sp);
    lw      $a1, 28($sp);
    lw      $a2, 32($sp);
    lw      $a3, 36($sp);
    addu    $t0, $a0, $a1;  # pSrc=ramStartAddress+ramSize;
    or      $t1, $0, $a2;   # pDst=stackTopAddress;
    or      $t2, $0, $a3;   # nBytes=stackSize;
    ;# copy
2:  lw      $t3, -4($t0);
    addiu   $t0, $t0, -4;
    sw      $t3, -4($t1);
    addiu   $t2, $t2, -4;
    bne     $t2, $0, 2b;
    addiu   $t1, $t1, -4;
    ;#
StackTestDone:
    ;# v0 has the result
    ; # restore the stack
    lw      $sp, 16($sp);
    lw      $ra, 20($sp);
    ;# clear up RAM contents
    lw      $a0, 24($sp);
    lw      $a1, 28($sp);
3:  sw      $0, 0($a0)
    addiu   $a1, $a1, -4;
    bne     $a1, $0, 3b;
    addiu   $a0, $a0, 4;
    ;
    jr $ra  
    addiu   $sp, $sp, MARCHC_STACK_UP_GUARD


.end SSL_32bitsFamily_RAM_STACKtest_MarchC

.set	at
.set	reorder



