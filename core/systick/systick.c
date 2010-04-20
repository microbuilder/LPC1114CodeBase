/**************************************************************************/
/*! 
    @file     systick.c
    @author   K. Townsend (microBuilder.eu)
    @date     22 March 2010
    @version  0.10

    @section DESCRIPTION

    Controls the 24-bit 'system tick' clock, which can be used as a
    generic timer or to control time sharing with an embedded real-time
    operating system (such as FreeRTOS).

    @section Example

    @code 
    #include "core/cpu/cpu.h"
    #include "core/systick/systick.h"

    void main (void)
    {
      cpuInit();

      // Start systick timer with one tick every 10ms
      systickInit(10);

      while(1)
      {
      }
    }
    @endcode

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

#include "systick.h"

volatile uint32_t msTicks;             // 1ms tick counter

void SysTick_Handler (void)
{
  msTicks++;
}

static uint32_t systickConfig(uint32_t ticks)
{ 
  // Check if 'ticks' is greater than maximum value
  if (ticks > SYSTICK_STRELOAD_MASK)
  {
    return (1);
  }
                     
  // Set reload register
  SYSTICK_STRELOAD  = (ticks & SYSTICK_STRELOAD_MASK) - 1;

  // Load the systick counter value
  SYSTICK_STCURR = 0;

  // Enable systick IRQ and timer
  SYSTICK_STCTRL = SYSTICK_STCTRL_CLKSOURCE |
                   SYSTICK_STCTRL_TICKINT |
                   SYSTICK_STCTRL_ENABLE;

  return (0);
}


void systickInit (uint32_t delayMs)
{
  systickConfig ((CFG_CPU_CCLK / 1000) * delayMs);
}

void systickDelay (uint32_t delayTicks) 
{
  uint32_t curTicks;
  curTicks = msTicks;

  if (curTicks > 0x00FFFFFF - delayTicks)
  {
    // Rollover will occur during delay
    while (msTicks >= curTicks)
    {
      while (msTicks < (delayTicks - (0x00FFFFFF - curTicks)));
    }      
  }
  else
  {
    while ((msTicks - curTicks) < delayTicks);
  }
}
