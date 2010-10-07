/**************************************************************************/
/*! 
    @file     eeprom.c
    @author   K. Townsend (microBuilder.eu)

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
#include <string.h>

#include "projectconfig.h"
#include "drivers/eeprom/mcp24aa/mcp24aa.h"
#include "eeprom.h"

static uint8_t buf[32];

/**************************************************************************/
/*! 
    @brief Reads 1 byte from EEPROM

    @param[in]  addr
                The 16-bit address to read from in EEPROM

    @return     An unsigned 8-bit value (uint8_t)
*/
/**************************************************************************/
uint8_t eepromReadU8(uint16_t addr)
{
  mcp24aaError_e error = MCP24AA_ERROR_OK;
  error = mcp24aaReadBuffer(addr, buf, sizeof(uint8_t));

  // ToDo: Handle any errors
  if (error) { };

  return buf[0];
}

/**************************************************************************/
/*! 
    @brief Reads 1 byte from EEPROM

    @param[in]  addr
                The 16-bit address to read from in EEPROM

    @return     A signed 8-bit value (int8_t)
*/
/**************************************************************************/
int8_t eepromReadS8(uint16_t addr)
{
  int8_t results;

  mcp24aaError_e error = MCP24AA_ERROR_OK;
  error = mcp24aaReadBuffer(addr, buf, sizeof(int8_t));
  
  // ToDo: Handle any errors
  if (error) { };

  memcpy(&results, buf, sizeof(int8_t));
  return results;
}

/**************************************************************************/
/*! 
    @brief Reads 2 bytes from EEPROM

    @param[in]  addr
                The 16-bit address to read from in EEPROM

    @return     A unsigned 16-bit value (uint16_t)
*/
/**************************************************************************/
uint16_t eepromReadU16(uint16_t addr)
{
  uint16_t results;

  mcp24aaError_e error = MCP24AA_ERROR_OK;
  error = mcp24aaReadBuffer(addr, buf, sizeof(uint16_t));
  
  // ToDo: Handle any errors
  if (error) { };

  memcpy(&results, buf, sizeof(uint16_t));
  return results;
}

/**************************************************************************/
/*! 
    @brief Reads 4 bytes from EEPROM

    @param[in]  addr
                The 16-bit address to read from in EEPROM

    @return     A unsigned 32-bit value (uint32_t)
*/
/**************************************************************************/
uint32_t eepromReadU32(uint16_t addr)
{
  uint32_t results;

  mcp24aaError_e error = MCP24AA_ERROR_OK;
  error = mcp24aaReadBuffer(addr, buf, sizeof(uint32_t));
  
  // ToDo: Handle any errors
  if (error) { };

  memcpy(&results, buf, sizeof(uint32_t));
  return results;
}

/**************************************************************************/
/*! 
    @brief Reads 4 bytes from EEPROM

    @param[in]  addr
                The 16-bit address to read from in EEPROM

    @return     A signed 32-bit value (int32_t)
*/
/**************************************************************************/
int32_t eepromReadS32(uint16_t addr)
{
  int32_t results;

  mcp24aaError_e error = MCP24AA_ERROR_OK;
  error = mcp24aaReadBuffer(addr, buf, sizeof(int32_t));
  
  // ToDo: Handle any errors
  if (error) { };

  memcpy(&results, buf, sizeof(int32_t));
  return results;
}

/**************************************************************************/
/*! 
    @brief Reads 8 bytes from EEPROM

    @param[in]  addr
                The 16-bit address to read from in EEPROM

    @return     A unsigned 64-bit value (uint64_t)
*/
/**************************************************************************/
uint64_t eepromReadU64(uint16_t addr)
{
  uint64_t results;

  mcp24aaError_e error = MCP24AA_ERROR_OK;
  error = mcp24aaReadBuffer(addr, buf, sizeof(uint64_t));
  
  // ToDo: Handle any errors
  if (error) { };

  memcpy(&results, buf, sizeof(uint64_t));
  return results;
}

/**************************************************************************/
/*! 
    @brief Writes a single byte to EEPROM

    @param[in]  addr
                The 16-bit address to read from in EEPROM
    @param[in]  data
                The 8-bit value to write to EEPROM

    @return     MCP24AA_ERROR_OK      - Write operation was successful
                MCP24AA_ERROR_I2CINIT - Unable to initialise I2C
                MCP24AA_ERROR_I2CBUSY - I2C busy
                MCP24AA_ERROR_ADDRERR - Address out of range

    Note:       MCP24AA_ERROR_OK is equal to 0, so you can test if any
                errors occured during write with:
                if (!(eepromWriteByte(0x1234, 0x00))) ...
*/
/**************************************************************************/
mcp24aaError_e eepromWriteByte(uint16_t addr, uint8_t data)
{
  // Write data at the supplied address
  return mcp24aaWriteByte(addr, data);
}

