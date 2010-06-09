/**************************************************************************/
/*! 
    @file     cmd_tbl.h
    @author   K. Townsend (microBuilder.eu)
    @date     22 March 2010
    @version  0.10

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

#ifndef __CMD_TBL_H__ 
#define __CMD_TBL_H__

#define CMD_COUNT (sizeof(cmd_tbl)/sizeof(cmd_t))

#include <stdio.h>

#ifdef CFG_INTERFACE_UART
#include "core/uart/uart.h"
#endif

// Function prototypes for the command table
void cmd_help(uint8_t argc, char **argv);         // handled by core/cmd/cmd.c
void cmd_hello(uint8_t argc, char **argv);
void cmd_sysinfo(uint8_t argc, char **argv);
#ifdef CFG_CHIBI
void cmd_chibi_addr(uint8_t argc, char **argv);
void cmd_chibi_ieeeaddr(uint8_t argc, char **argv);
void cmd_chibi_tx(uint8_t argc, char **argv);
#endif
#ifdef CFG_I2CEEPROM
void cmd_i2ceeprom_read(uint8_t argc, char **argv);
void cmd_i2ceeprom_write(uint8_t argc, char **argv);
#endif
#ifdef CFG_LM75B
void cmd_lm75b_gettemp(uint8_t argc, char **argv);
#endif

/**************************************************************************/
/*! 
    Command list for the command-line interpreter and the name of the
    corresponding method that handles the command.

    Note that a trailing ',' is required on the last entry, which will
    cause a NULL entry to be appended to the end of the table.
*/
/**************************************************************************/
cmd_t cmd_tbl[] = 
{
  // command name, min args, max args, function name, command description, syntax description
  { "help",           0, 0, cmd_help              , "Displays a list of all available commands"           , "'help' has no parameters" },
  { "hello",          0, 1, cmd_hello             , "Displays 'Hello World!'"                             , "'hello [<name>]'" },
  { "sysinfo",        0, 0, cmd_sysinfo           , "Displays current system configuration settings"      , "'sysinfo' has no parameters" },

  #ifdef CFG_CHIBI
  { "chb-addr",       0, 1, cmd_chibi_addr        , "Chibi - Gets/sets the 16-bit address"                , "'chb-addr [<1-65534>|<OxFFFE>]'" },
  { "chb-send",       2, 2, cmd_chibi_tx          , "Chibi - Transmits the supplied text/value"           , "'chb-tx <destaddr> <message>'" },
  #endif

  #ifdef CFG_I2CEEPROM
  { "eeprom-read",    1, 1, cmd_i2ceeprom_read    , "EEPROM - Reads one byte from the specified address"  , "'eeprom-read <addr>'" },
  { "eeprom-write",   2, 2, cmd_i2ceeprom_write   , "EEPROM - Writes one byte to the specified address"   , "'eeprom-write <addr> <value>'" },
  #endif

  #ifdef CFG_LM75B
  { "lm75b-gettemp",  0, 0, cmd_lm75b_gettemp     , "LM75B - Current temperature in degrees celsius"      , "'lm75b-gettemp'" },
  #endif
};

#endif