/*******************************************************************************
  Class B Library Implementation File

  Summary:
    This file contains the implementation for 
    the Class B Safety Software Library Flash CRC16 calculation
    for PIC32MX MCUs.
    
*******************************************************************************/
/*******************************************************************************
FileName:       SSL_FlashTest_CRC16.c
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

#include "ClassB/SSL_Flash_CRC.h"


// definitions
#define     FLASH_CRC16_MASK    ((1L<<16)-1)    		// mask to retain the useful CRC result for 16 bit polynomial
#define     FLASH_CRC16_MSB     (1L<<(16-1))			// mask to obtain the MSb-1, transport to the MSb.


// prototype
unsigned int Flash_CRC16(unsigned char* pBuff, unsigned int crcPoly, unsigned int crcReg, unsigned int bSize);


/*******************************************************************************
  Function:
    unsigned int SSL_32bitsFamily_Flashtest_CRC16(char* startAddress, char* endAddress, unsigned int crcPoly, unsigned int crcSeed)

  Summary:
    The Flash CRC16 test implements the periodic modified checksum
    H.2.19.3.1 as defined by the IEC 60730 standard.



    

  Description:
    This routine  detects the single bit Faults in the invariable memory.
    The invariable memory in a system, such as Flash and EEPROM memory,
    contains data that is not intended to vary during the program execution.


    The test calculates the 16 bit CRC of the supplied memory area
    using the standard LFSR (Linear Feedback Shift Register) implementation.
    It calculates over the memory area between the startAddress and endAddress and returns the CRC Value.
    The 16 bit CRC is calculated using the supplied generator polynomial and initial seed.
    Different generator polynomials can be used as indicated above.
                                                                             
  Precondition:
    None.

  Parameters:
    startAddress    - start Address of the memory area to start CRC calculation from

    endAddress      - final address for which the CRC is calculated

    crcPoly         - the generator polynomial to be used.
                      One of the standard supplied polynomials can be used
                      as well as other user defined ones.

    crcSeed         - the initial value in the CRC LFSR.
                      The usual recommended value is 0xffff.


                          
  Returns:
    The value of the calculated CRC over the specified memory area.

  Example:
    <code>
    unsigned int crcRes=SSL_32bitsFamily_Flashtest_CRC16(startAddress, endAddress, CRC_16_GEN_POLY, 0xffff);
    if(crcRes==prevCalculatedCrc)
    {
        // process test success
    }
    else
    {
        // process tests failure: the CRC of the memory area has changed.
    }
    </code>

  Remarks:
    This is a non-destructive memory test.

    The startAddress and endAdress over which the CRC value is calculated are
    PIC32 variant and application dependent. They are run-time parameters.

    Refer to the AN1229 for details regarding the SSL_32bitsFamily_RAMtest_CheckerBoard()
    and the Class B Software Library.
  *****************************************************************************/

unsigned int SSL_32bitsFamily_Flashtest_CRC16(char* startAddress, char* endAddress, unsigned int crcPoly, unsigned int crcSeed)
{
    return Flash_CRC16((unsigned char*)startAddress, crcPoly, crcSeed, endAddress-startAddress+1);
}

  
/*********************************************************************
 * Function:        unsigned int Flash_CRC16(unsigned char* pBuff, unsigned int crcPoly, unsigned int crcReg, unsigned int bSize)
 *
 * PreCondition:    pBuff valid pointer
 * 
 * Input:           - pBuff:    buffer to calculate CRC over
 *                  - crcPoly:  the generator polynomial to be used
 *                  - crcReg:   initial value of the CRC LFSR (seed)
 * 					- bSize:    buffer size, bytes
 * 
 * Output:          value of the CRC
 * 
 * Side Effects:    None
 * 
 * Overview:        Shifts bytes through the CRC shift register.
 * 
 * Note:            Simple (and slow) CRC16 calculation directly based
 *                  on the hardware LFSR implementation.
 *                  
 ********************************************************************/
unsigned int Flash_CRC16(unsigned char* pBuff, unsigned int crcPoly, unsigned int crcReg, unsigned int bSize)
{
	while(bSize--)
	{
		int		ix;
		unsigned char b=*pBuff++;
		int		mask=0x80;
		
		for(ix=0; ix<8; ix++)
		{
			int	fback=(crcReg&FLASH_CRC16_MSB)!=0;  // see if there's transport from the MSb of the LFSR
            
			crcReg<<=1;                             // shift the LFSR
            
            // feed the data MSb into LFSR LSb
			if((b&mask)!=0)
			{
				crcReg|=1;  // set the LSb of the LFSR
			}
           
            // apply the polynomial xor
            // if there's transport out of the LFSR MSb 
			if(fback)
			{
				crcReg^=crcPoly;
			}
            
			mask>>=1;               // move to the next bit in incoming data
		}
	}

	return crcReg&FLASH_CRC16_MASK;    
}


