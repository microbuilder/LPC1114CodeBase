/**************************************************************************/
/*! 
    @file     cmd_deepsleep.c
    @author   K. Townsend (microBuilder.eu)

    @brief    Code to execute for cmd_deepsleep in the 'core/cmd'
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
#include "commands.h"

#include "core/pmu/pmu.h"

/**************************************************************************/
/*! 
    Puts the device into deep sleep
*/
/**************************************************************************/
void cmd_deepsleep(uint8_t argc, char **argv)
{
  // ToDo: Some peripherals may need to be manually disabled for the
  // lowest possible power consumption in deep sleep mode

  // Enter deep sleep mode (wakeup after ~10 seconds)
  // Note that the exact delay is variable since the internal WDT oscillator
  // is used for lowest possible power consumption and because it requires
  // no external components, but it only has +-/40% accuracy
  printf("Entering Deep Sleep mode for 10 seconds%s", CFG_PRINTF_NEWLINE);
  pmuDeepSleep(10);

  // On wakeup, the WAKEUP interrupt will be fired, which is handled
  // by WAKEUP_IRQHandler in 'core/pmu/pmu.c'.  This will set the CPU
  // back to an appropriate state, and execution will be returned to
  // the point that it left off before deep sleep mode was entered.
  printf("Woke up from deep sleep");
}
