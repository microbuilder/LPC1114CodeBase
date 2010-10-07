/**************************************************************************/
/*! 
    @file     cmd_i2ceeprom_write.c
    @author   K. Townsend (microBuilder.eu)

    @brief    Code to execute for cmd_i2ceeprom_write in the 'core/cmd'
              command-line interpretter.

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2010, microBuilder SARL
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#include <stdio.h>

#include "projectconfig.h"
#include "core/cmd/cmd.h"
#include "commands.h"       // Generic helper functions

#ifdef CFG_I2CEEPROM
  #include "drivers/eeprom/mcp24aa/mcp24aa.h"
  #include "eeprom.h"

/**************************************************************************/
/*! 
    Writes a single byte at the supplied EEPROM address
*/
/**************************************************************************/
void cmd_i2ceeprom_write(uint8_t argc, char **argv)
{
  uint16_t addr;
  uint8_t val;

  // Try to convert supplied address to an integer
  int32_t addr32;
  getNumber (argv[0], &addr32);
  
  // Check for invalid values (getNumber may complain about this as well)
  if (addr32 < 0 || addr32 > MCP24AA_MAXADDR)
  {
    printf("Address out of range: Value from 0-%d or 0x0000-0x%04X required.%s", MCP24AA_MAXADDR, MCP24AA_MAXADDR, CFG_PRINTF_NEWLINE);
    return;
  }

  // If Chibi is enabled, make sure we are not overwriting the short or IEEE address in EEPROM
  #ifdef CFG_CHIBI
  if ((addr32 >= CFG_CHIBI_EEPROM_IEEEADDR) && (addr32 <= CFG_CHIBI_EEPROM_IEEEADDR + 7))
  {
    printf("Reserved Address: 0x%04X to 0x%04X is reserved for Chibi IEEE address%s", CFG_CHIBI_EEPROM_IEEEADDR, CFG_CHIBI_EEPROM_IEEEADDR + 7, CFG_PRINTF_NEWLINE);
    return;
  }
  if ((addr32 >= CFG_CHIBI_EEPROM_SHORTADDR) && (addr32 <= CFG_CHIBI_EEPROM_SHORTADDR + 1))
  {
    printf("Reserved Address: 0x%04X to 0x%04X is reserved for Chibi address%s", CFG_CHIBI_EEPROM_SHORTADDR, CFG_CHIBI_EEPROM_SHORTADDR + 1, CFG_PRINTF_NEWLINE);
    return;
  }
  #endif

  // Address seems to be OK
  addr = (uint16_t)addr32;

  // Try to convert supplied data to an integer
  int32_t val32;
  getNumber (argv[1], &val32);
  
  // Check for invalid values (getNumber may complain about this as well)
  if (val32 < 0 || val32 > 0xFF)
  {
    printf("Invalid Data: Value from 0-255 or 0x00-0xFF required.%s", CFG_PRINTF_NEWLINE);
    return;
  }

  // Data seems to be OK
  val = (uint8_t)val32;

  // Instantiate error message placeholder
  mcp24aaError_e error = MCP24AA_ERROR_OK;

  // Write data at supplied address
  error = mcp24aaWriteByte(addr, val);
  if (error)
  {
    // Handle any errors
    switch (error)
    {
      case (MCP24AA_ERROR_I2CINIT):
        printf("Error: Unable to initialised I2C%s", CFG_PRINTF_NEWLINE);
        return;
      case (MCP24AA_ERROR_ADDRERR):
        printf("Error: Supplied address is out of range%s", CFG_PRINTF_NEWLINE);
        return;
      default:
        break;
    }
  }

  // Write successful
  printf("EEPROM: 0x%02X written at address 0x%04X%s", val, addr, CFG_PRINTF_NEWLINE);
}

#endif
