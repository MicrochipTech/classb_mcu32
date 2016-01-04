/*******************************************************************************
  Class B Library implementation file

  Summary:
    This file contains the implementation for the
    the Class B Safety Software Library CPU Register Tests
    for PIC32MX MCUs.
        
*******************************************************************************/
/*******************************************************************************
FileName:       SSL_CpuRegisterTest.s
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
.equ   CPU_REGISTER_TEST_PASS,  1

.equ  CPU_REGISTER_TEST_FAIL,  0

.global SSL_32bitsFamily_CPU_RegisterTest


#; test used defines and storage 
.equ  MASK_AA_LOW,  0x0000AAAA
.equ  MASK_55_LOW,  0x00005555


.text

ROM_AA_WORD:
	.word	0xaaaaaaaa;
ROM_55_WORD:
	.word	0x55555555;


/* implementation */
    
# define a macro to perform register 0 check
# in case of error a jump to err_label is performed
# make sure reg 0 is 0x0
# xori is used and a flash value
# temporary register also used

.macro check_r0 temp_reg err_label
    lui     \temp_reg,  0xffff;             # set hi half, lower low half, don't use $0    
    rotr    \temp_reg, \temp_reg, 16;       # swap halves
    xori    \temp_reg, \temp_reg, 0xffff;   # make the register 0

    bne     \temp_reg, $0, \err_label
    nop;
    #; finally
    bltz    $0, \err_label
    nop;
    bgtz    $0, \err_label;
    nop;
#; everything seems to be ok.
    
.endm


#; define a macro to perform the basic register check
#; in case of error a jump to err_label is performed
#; since comparing 2 regs loaded with the same value doesn't say
#; anything when both have same stuck bits a different approach is taken:
#; we make sure the value is all aa's or 55's
#; use 16 bit xori operations to not rely on another register
#; we use just register 0 which should be 0!

.macro check_register check_reg err_label
    la      \check_reg, ROM_AA_WORD;
    lw      \check_reg, 0(\check_reg);              # all 0xaa    
    xori    \check_reg, \check_reg, MASK_AA_LOW;    # clear low half
    rotr    \check_reg, \check_reg, 16;
    xori    \check_reg, \check_reg, MASK_AA_LOW;    # clear hi half
    bne     \check_reg, $0, \err_label
    nop;
    #; same for the other 0x55
    la      \check_reg, ROM_55_WORD;
    lw      \check_reg, 0(\check_reg);              # all 0x55    
    xori    \check_reg, \check_reg, MASK_55_LOW;    # clear low half
    rotr    \check_reg, \check_reg, 16;
    xori    \check_reg, \check_reg, MASK_55_LOW;    # clear hi half
    bne     \check_reg, $0, \err_label
    nop;
.endm


    
.set    noreorder
.set    noat

.text

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
    * CPU_REGISTER_TEST_PASS    - The test passed. CPU registers have not been detected to have stuck bits. 
    * CPU_REGISTER_TEST_FAIL    - The test failed. Some CPU register(s) has been detected to have stuck bits. 

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
.ent    SSL_32bitsFamily_CPU_RegisterTest
SSL_32bitsFamily_CPU_RegisterTest:

# preserve all registers required by the compiler for a safe return
# interrupts must be disabled as sp, k0 and k1 cannot be tested otherwise
    addiu   $sp, $sp, -44
    sw      $fp, 0($sp)
    sw      $gp, 4($sp)
    sw      $ra, 8($sp)
    sw      $s0, 12($sp)
    sw      $s1, 16($sp)
    sw      $s2, 20($sp)
    sw      $s3, 24($sp)
    sw      $s4, 28($sp)
    sw      $s5, 32($sp)
    sw      $s6, 36($sp)
    sw      $s7, 40($sp)


#;*********************************
#; Test $0 register
    check_r0    $at CpuTestError

#;*********************************
#; Test $at register (1)
   check_register $at CpuTestError

#;*********************************
#; Test $v0-$v1 registers (2-3)
   check_register $v0 CpuTestError
   check_register $v1 CpuTestError 

#;*********************************
#; Test $a0-$a3 registers (4-7)
   check_register $a0 CpuTestError 
   check_register $a1 CpuTestError 
   check_register $a2 CpuTestError 
   check_register $a3 CpuTestError 

#;*********************************
#; Test $t0-$t7 registers (8-15)
   check_register $t0 CpuTestError 
   check_register $t1 CpuTestError 
   check_register $t2 CpuTestError 
   check_register $t3 CpuTestError 
   check_register $t4 CpuTestError 
   check_register $t5 CpuTestError 
   check_register $t6 CpuTestError 
   check_register $t7 CpuTestError 

#;*********************************
#; Test $s0-$s7 registers (16-23)
   check_register $s0 CpuTestError 
   check_register $s1 CpuTestError 
   check_register $s2 CpuTestError 
   check_register $s3 CpuTestError 
   check_register $s4 CpuTestError 
   check_register $s5 CpuTestError 
   check_register $s6 CpuTestError 
   check_register $s7 CpuTestError 

#;*********************************
#; Test $t8-$t9 registers (24-25)
   check_register $t8 CpuTestError 
   check_register $t9 CpuTestError 

#;*********************************
#; Test $k0-$k1 registers (26-27) 
   check_register $k0 CpuTestError
   check_register $k1 CpuTestError 

#;*********************************
#; Test $gp, $fp, $ra registers (28, 30, 31) 
   check_register $gp CpuTestError
   check_register $fp CpuTestError 
   check_register $ra CpuTestError

#;*********************************
# lastly registers 29 $sp needs to be preserved (saved in $t0)
    move $t0, $sp;          # save $sp       
    check_register $sp SpTestError
    move $sp, $t0;          # restore $sp 

CpuTestSuccess:
    b       CpuTestDone;     # done
    addiu   $v0, $zero, CPU_REGISTER_TEST_PASS   #return Success code (Branch slot)
    
SpTestError:
    move $sp, $t0           # restore $sp
    #;  fall through
CpuTestError:
    addiu   $v0, $zero, CPU_REGISTER_TEST_FAIL   #return Error code

CpuTestDone:
#;  restore saved regs
    lw      $fp, 0($sp)
    lw      $gp, 4($sp)
    lw      $ra, 8($sp)
    lw      $s0, 12($sp)
    lw      $s1, 16($sp)
    lw      $s2, 20($sp)
    lw      $s3, 24($sp)
    lw      $s4, 28($sp)
    lw      $s5, 32($sp)
    lw      $s6, 36($sp)
    lw      $s7, 40($sp)
    addiu   $sp, $sp, 44

#   return
    jr $ra  
    nop


.end SSL_32bitsFamily_CPU_RegisterTest

.set	at
.set	reorder



