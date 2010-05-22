/**************************************************************************/
/*! 
    @file     commands.c
    @author   K. Townsend (microBuilder.eu)
    @date     22 March 2010
    @version  0.10

    @brief    Entry point for all commands in the 'core/cmd' command-line
              interpretter.  Every menu item defined in cmd_tbl.h points
              to a method that should be located here for convenience
              sake.  (The only exception is the 'help', which exists in
              any project and is handled directly by core/cmd/cmd.c). All
              methods have exactly the same signature (argc + argv).

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

#include "core/cmd/cmd.h"

#ifdef CFG_CHIBI
  #include "drivers/chibi/chb.h"
#endif

/**************************************************************************/
/*! 
    'hello' command handler
*/
/**************************************************************************/
void cmd_hello(uint8_t argc, char **argv)
{
  if (argc > 0)
  {
    printf("Hello %s", argv[0]);
  }
  else
  {
    printf("Hello World!%s", CFG_INTERFACE_NEWLINE);
  }
}

/**************************************************************************/
/*! 
    'sysinfo' command handler
*/
/**************************************************************************/
void cmd_sysinfo(uint8_t argc, char **argv)
{
  printf("%-30s : %d Hz %s", "Core System Clock", CFG_CPU_CCLK, CFG_INTERFACE_NEWLINE);
  printf("%-30s : %d mS %s", "Systick Timer Delay", CFG_SYSTICK_DELAY_IN_MS, CFG_INTERFACE_NEWLINE);
  printf("%-30s : %d BPS %s", "UART Baud Rate", CFG_UART_BAUDRATE, CFG_INTERFACE_NEWLINE);

  #ifdef CFG_CHIBI
    chb_pcb_t *pcb = chb_get_pcb();
    printf("%-30s : %s%s", "Wireless Frequency", "868 MHz", CFG_INTERFACE_NEWLINE);
    printf("%-30s : 0x%04X%s", "Wireless Node Address", pcb->src_addr, CFG_INTERFACE_NEWLINE);
  #endif

  // printf("%-30s : %s", "<Property Name>", CFG_INTERFACE_NEWLINE);
}

