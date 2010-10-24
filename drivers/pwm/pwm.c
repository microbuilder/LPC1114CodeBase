/**************************************************************************/
/*! 
    @file     pwm.c
    @author   K. Townsend (microBuilder.eu)

    @brief    Simple PWM example that can be used to control a stepper
              motor, dim an LED, etc.

    @section Example

    @code 
    #include "drivers/pwm/pwm.h"
    ...

    // Initialises PWM output on 16-bit Timer 1 and
    // sets MAT0 (P1.9) as output
    pwmInit();

    // Setup the pulse-width and duty-cycle
    pwmSetDutyCycle(50);                  // Set 50% duty cycle
    pwmSetFrequencyInMicroseconds(100);   // 100 millisecond pulse width

    // Enable to PWM output
    pwmStart();

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
#include "pwm.h"

static uint32_t pwmPulseWidth = CFG_CPU_CCLK / 1000;
static uint32_t pwmPercentage = 50;

/**************************************************************************/
/*! 
    Initialises 16-bit Timer 1, and configures the MAT0 output (pin 1.9) 
    to send the PWM output signal.
*/
/**************************************************************************/
void pwmInit(void)
{
  /* Enable the clock for CT16B1 */
  SCB_SYSAHBCLKCTRL |= (SCB_SYSAHBCLKCTRL_CT16B1);

  /* Configure PIO1.9 as Timer1_16 MAT0 Output */
  IOCON_PIO1_9 &= ~IOCON_PIO1_9_FUNC_MASK;
  IOCON_PIO1_9 |= IOCON_PIO1_9_FUNC_CT16B1_MAT0;  

  /* Set default pulse width (MR3)*/
  TMR_TMR16B1MR3 = pwmPulseWidth;

  /* Set default duty cycle (MR0) */
  TMR_TMR16B1MR0 = (pwmPulseWidth * (100 - pwmPercentage)) / 100;

  /* Configure match control register to reset on MR3 */
  TMR_TMR16B1MCR = (TMR_TMR16B1MCR_MR3_RESET_ENABLED);

  /* External Match Register Settings for PWM */
  TMR_TMR16B1EMR = TMR_TMR16B1EMR_EMC0_TOGGLE | TMR_TMR16B1EMR_EM0;

  /* Enable PWM0 and PWM3 */
  TMR_TMR16B1PWMC = TMR_TMR16B1PWMC_PWM0_ENABLED | TMR_TMR16B1PWMC_PWM3_ENABLED;
}

/**************************************************************************/
/*! 
    Starts the PWM output
*/
/**************************************************************************/
void pwmStart(void)
{
  /* Enable Timer1 */
  TMR_TMR16B1TCR = TMR_TMR16B1TCR_COUNTERENABLE_ENABLED;
}

/**************************************************************************/
/*! 
    Stops the PWM output
*/
/**************************************************************************/
void pwmStop(void)
{
  /* Disable Timer1 */
  TMR_TMR16B1TCR = TMR_TMR16B1TCR_COUNTERENABLE_DISABLED;  
}

/**************************************************************************/
/*! 
    Sets the signal's duty cycle in percent (1-100).
*/
/**************************************************************************/
int pwmSetDutyCycle(uint32_t percentage)
{
  if ((percentage < 1) || (percentage > 100))
  {
    /* Duty Cycle must be a value between 1 and 100 */
    return -1;
  }

  /* Set Duty Cycle (MR0) */
  TMR_TMR16B1MR0 = (pwmPulseWidth * (100 - (pwmPercentage = percentage))) / 100;

  return 0;
}

/**************************************************************************/
/*! 
    Sets the signal's frequency/pulse-width to the specified number
    of ticks.
*/
/**************************************************************************/
int pwmSetFrequencyInTicks(uint16_t frequency)
{
  if (frequency < 1)
  {
    return -1;
  }

  /* Set Pulse Width (MR3)*/
  TMR_TMR16B1MR3 = (pwmPulseWidth = frequency);

  /* Adjust Duty Cycle (MR0) */
  TMR_TMR16B1MR0 = (pwmPulseWidth * (100 - pwmPercentage)) / 100;

  return 0;  
}

/**************************************************************************/
/*! 
    Sets the signal's frequency/pulse-width to the specified number
    of microseconds.
*/
/**************************************************************************/
int pwmSetFrequencyInMicroseconds(uint16_t frequency)
{
  if (frequency < 1)
  {
    return -1;
  }

  uint32_t ticks = (((CFG_CPU_CCLK/SCB_SYSAHBCLKDIV) / 1000000) * frequency);
  if (ticks > 0xFFFF)
  {
    /* Delay exceeds the upper limit for the 16-bit timer */
    return -1;
  }

  /* Set Pulse Width (MR3)*/
  TMR_TMR16B1MR3 = (pwmPulseWidth = ticks);

  /* Adjust Duty Cycle (MR0) */
  TMR_TMR16B1MR0 = (pwmPulseWidth * (100 - pwmPercentage)) / 100;

  return 0;  
}


