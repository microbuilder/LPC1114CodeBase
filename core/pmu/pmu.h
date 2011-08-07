/**************************************************************************/
/*! 
    @file     pmu.h
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

#ifndef __PMU_H__
#define __PMU_H__

#include "projectconfig.h"

/**************************************************************************/
/*! 
    Pointers to the power profiles functions in LPC1100L series MCUs.
*/
/**************************************************************************/
typedef struct _PWRD 
{
  void (*set_pll)(unsigned int cmd[], unsigned int resp[]);
  void (*set_power)(unsigned int cmd[], unsigned int resp[]);
} PWRD;

/**************************************************************************/
/*! 
    ROM entry table ... this is used to access power profiles in ROM.
*/
/**************************************************************************/
typedef	struct _ROM 
{
   const unsigned p_usbd;
   const unsigned p_clib;
   const unsigned p_cand;
   const PWRD * pPWRD;
   const unsigned p_dev1;
   const unsigned p_dev2;
   const unsigned p_dev3;
   const unsigned p_dev4; 
}  ROM;

/**************************************************************************/
/*! 
    Indicates a specific LPC1100L power-profile (only relevant to 'L'
    series chips).
*/
/**************************************************************************/
typedef enum pmuPowerProfile_e
{
  pmuPowerProfile_Default = 0,
  pmuPowerProfile_Performance = 1,
  pmuPowerProfile_Efficiency = 2,
  pmuPowerProfile_LowCurrent = 3
} 
pmuPowerProfile_t;

void WAKEUP_IRQHandler( void );
void pmuInit( void );
void pmuSleep( void );
void pmuDeepSleep(uint32_t wakeupSeconds);
void pmuPowerDown( void );

#endif
