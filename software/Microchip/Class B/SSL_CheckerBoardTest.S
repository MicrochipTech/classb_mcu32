/*******************************************************************************
  Class B Library implementation file

  Summary:
    This file contains the implementation for the
    the Class B Safety Software Library RAM Checkerboard Tests
    for PIC32MX MCUs.
        
*******************************************************************************/
/*******************************************************************************
FileName:       SSL_CheckerBoardTest.s
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
.equ   CB_TEST_PASS,  1

.equ   CB_TEST_FAIL,  0

.global SSL_32bitsFamily_RAMtest_CheckerBoard

/* implementation */

.set	noat
.set	noreorder

.text

;# saves the test space area ($a0) in t0-t7, s0-s7
.macro  save_area
        lw      $t0, 0($a0)
        lw      $t1, 4($a0)
        lw      $t2, 8($a0)
        lw      $t3, 12($a0)
        lw      $t4, 16($a0)
        lw      $t5, 20($a0)
        lw      $t6, 24($a0)
        lw      $t7, 28($a0)
        lw      $s0, 32($a0)
        lw      $s1, 36($a0)
        lw      $s2, 40($a0)
        lw      $s3, 44($a0)
        lw      $s4, 48($a0)
        lw      $s5, 52($a0)
        lw      $s6, 56($a0)
        lw      $s7, 60($a0)
.endm

;# restores the test space area ($a0) from t0-t7, s0-s7
.macro  restore_area
        sw      $t0, 0($a0)
        sw      $t1, 4($a0)
        sw      $t2, 8($a0)
        sw      $t3, 12($a0)
        sw      $t4, 16($a0)
        sw      $t5, 20($a0)
        sw      $t6, 24($a0)
        sw      $t7, 28($a0)
        sw      $s0, 32($a0)
        sw      $s1, 36($a0)
        sw      $s2, 40($a0)
        sw      $s3, 44($a0)
        sw      $s4, 48($a0)
        sw      $s5, 52($a0)
        sw      $s6, 56($a0)
        sw      $s7, 60($a0)
.endm

;# fills the test area ($a0) with the pattern $a2/$a3
.macro  fill_pattern
        sw      $a2, 0($a0)
        sw      $a3, 4($a0)
        sw      $a2, 8($a0)
        sw      $a3, 12($a0)
        sw      $a2, 16($a0)
        sw      $a3, 20($a0)
        sw      $a2, 24($a0)
        sw      $a3, 28($a0)
        sw      $a2, 32($a0)
        sw      $a3, 36($a0)
        sw      $a2, 40($a0)
        sw      $a3, 44($a0)
        sw      $a2, 48($a0)
        sw      $a3, 52($a0)
        sw      $a2, 56($a0)
        sw      $a3, 60($a0)
.endm

;# reads back the test area ($a0) and expects $a2/$a3 sequentially
;# if error found sets the error code in $v0 and jumps to label
;# else sets success code in $v0
;# uses $v1
.macro  read_pattern   errLabel
        lw      $v1, 0($a0)
        bne     $v1, $a2, \errLabel
        ori     $v0, $0, CB_TEST_FAIL;    
        lw      $v1, 4($a0)
        bne     $v1, $a3, \errLabel
        ori     $v0, $0, CB_TEST_FAIL;    
        lw      $v1, 8($a0)
        bne     $v1, $a2, \errLabel
        ori     $v0, $0, CB_TEST_FAIL;    
        lw      $v1, 12($a0)
        bne     $v1, $a3, \errLabel
        ori     $v0, $0, CB_TEST_FAIL;    
        lw      $v1, 16($a0)
        bne     $v1, $a2, \errLabel
        ori     $v0, $0, CB_TEST_FAIL;    
        lw      $v1, 20($a0)
        bne     $v1, $a3, \errLabel
        ori     $v0, $0, CB_TEST_FAIL;    
        lw      $v1, 24($a0)
        bne     $v1, $a2, \errLabel
        ori     $v0, $0, CB_TEST_FAIL;    
        lw      $v1, 28($a0)
        bne     $v1, $a3, \errLabel
        ori     $v0, $0, CB_TEST_FAIL;    
        lw      $v1, 32($a0)
        bne     $v1, $a2, \errLabel
        ori     $v0, $0, CB_TEST_FAIL;    
        lw      $v1, 36($a0)
        bne     $v1, $a3, \errLabel
        ori     $v0, $0, CB_TEST_FAIL;    
        lw      $v1, 40($a0)
        bne     $v1, $a2, \errLabel
        ori     $v0, $0, CB_TEST_FAIL;    
        lw      $v1, 44($a0)
        bne     $v1, $a3, \errLabel
        ori     $v0, $0, CB_TEST_FAIL;    
        lw      $v1, 48($a0)
        bne     $v1, $a2, \errLabel
        ori     $v0, $0, CB_TEST_FAIL;    
        lw      $v1, 52($a0)
        bne     $v1, $a3, \errLabel
        ori     $v0, $0, CB_TEST_FAIL;    
        lw      $v1, 56($a0)
        bne     $v1, $a2, \errLabel
        ori     $v0, $0, CB_TEST_FAIL;    
        lw      $v1, 60($a0)
        bne     $v1, $a3, \errLabel
        ori     $v0, $0, CB_TEST_FAIL;
        ;# success
        ori     $v0, $0, CB_TEST_PASS;
.endm        




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

.ent	SSL_32bitsFamily_RAMtest_CheckerBoard

# int _SSL_32bitsFamily_RAMtest_CheckerBoard( int *startAddress, int ramSize)
# input:
# 	$a0 startAddress        must be word aligned
# 	$a1 ramSize             must be a multiple of 64
# output:
# 	$v0                     return value
# USES:
#       $t0-$t7, $s0-$s7 - RAM save/restore
#       $a2, $a3 - pattern registers
#       $v1 - temporary register
#
SSL_32bitsFamily_RAMtest_CheckerBoard:


        addiu   $sp, $sp, -32;  # register save space
        ;# save regs in the stack frame
        sw      $s0, 0($sp);
        sw      $s1, 4($sp);
        sw      $s2, 8($sp);
        sw      $s3, 12($sp);
        sw      $s4, 16($sp);
        sw      $s5, 20($sp);
        sw      $s6, 24($sp);
        sw      $s7, 28($sp);
    

_loop:  ;# main test loop

        ;# load the test constants
        li      $a2, 0x55555555;
        li      $a3, 0xaaaaaaaa;

        save_area;                  # save the test space

        fill_pattern;               # fill with the pattern
    
        read_pattern _restore;      # read back the pattern data and restore if failed
       
        ;# passed 
        ;# switch the pattern
        or      $v1, $0, $a2;
        or      $a2, $0, $a3;
        or      $a3, $0, $v1;
        
        fill_pattern;               # fill with the switched pattern      
        read_pattern _restore;      # read back the pattern data and restore if failed
    
_restore:
        restore_area;               # restore the test space; $v0 holds the result
        ;# if suceeded we continue
        ;# else just return
        ori     $v1, $0, CB_TEST_PASS
        bne     $v0, $v1, _done;
        nop;
        addiu   $a1, $a1, -64
        bne     $a1, $0, _loop;
        addiu   $a0, $a0, 64;
        

_done:  ;# restore stack frame 
        lw      $s0, 0($sp);
        lw      $s1, 4($sp);
        lw      $s2, 8($sp);
        lw      $s3, 12($sp);
        lw      $s4, 16($sp);
        lw      $s5, 20($sp);
        lw      $s6, 24($sp);
        lw      $s7, 28($sp);
        
        jr      $ra;
        addiu   $sp, $sp, 32;


.end SSL_32bitsFamily_RAMtest_CheckerBoard


.set	at
.set	reorder
    
