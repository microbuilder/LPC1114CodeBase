/**************************************************************************/
/*! 
    @file     pmu.c
    @author   K. Townsend (microBuilder.eu)
    @date     22 March 2010
    @version  0.10

    @section DESCRIPTION

    Controls the power management features of the LPC1114, allowing you
    to enter sleep/deep-sleep or deep power-down mode.

    For examples of how to enter either mode, see the comments for the
    functions pmuSleep(), pmuDeepSleep() and pmuPowerDown().
	
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

#include "core/gpio/gpio.h"
#include "core/cpu/cpu.h"
#include "core/timer32/timer32.h"
#include "pmu.h"

#define PMU_WDTCLOCKSPEED_HZ 10000

/**************************************************************************/
/*! 
    Wakeup interrupt handler
*/
/**************************************************************************/
void WAKEUP_IRQHandler(void)
{
  uint32_t regVal;

  // Disable the deep sleep timer
  TMR_TMR32B0TCR = TMR_TMR32B0TCR_COUNTERENABLE_DISABLED;

  /* This handler takes care of all the port pins if they
  are configured as wakeup source. */
  regVal = SCB_STARTSRP0;
  if (regVal != 0)
  {
    SCB_STARTRSRP0CLR = regVal;
  }

  // Reconfigure system clock/PLL
  cpuPllSetup(CPU_MULTIPLIER_1);

  // Reconfigure CT32B0
  timer32Init(0, TIMER32_DEFAULTINTERVAL);
  timer32Enable(0);

  /* See tracker for bug report. */
  __asm volatile ("NOP");

  return;
}

/**************************************************************************/
/*! 
    Setup the clock for the watchdog timer.  The default setting is 10kHz.
*/
/**************************************************************************/
static void pmuWDTClockInit (void)
{
  /* Configure watchdog clock */
  /* Freq. = 0.5MHz, div = 50: WDT_OSC = 10kHz  */
  SCB_WDTOSCCTRL = SCB_WDTOSCCTRL_FREQSEL_0_5MHZ | 
                   SCB_WDTOSCCTRL_DIVSEL_DIV50;

  /* Set clock source (use internal oscillator) */
  // SCB_WDTCLKSEL = SCB_WDTCLKSEL_SOURCE_INPUTCLOCK;
  SCB_WDTCLKSEL = SCB_WDTCLKSEL_SOURCE_INTERNALOSC;
  SCB_WDTCLKUEN = SCB_WDTCLKUEN_UPDATE;
  SCB_WDTCLKUEN = SCB_WDTCLKUEN_DISABLE;
  SCB_WDTCLKUEN = SCB_WDTCLKUEN_UPDATE;

  /* Wait until updated */
  while (!(SCB_WDTCLKUEN & SCB_WDTCLKUEN_UPDATE));

  /* Set divider */
  SCB_WDTCLKDIV = SCB_WDTCLKDIV_DIV1;

  /* Enable WDT clock */
  SCB_PDRUNCFG &= ~(SCB_PDRUNCFG_WDTOSC);

  // Switch main clock to WDT output
  SCB_MAINCLKSEL = SCB_MAINCLKSEL_SOURCE_WDTOSC;
  SCB_MAINCLKUEN = SCB_MAINCLKUEN_UPDATE;       // Update clock source
  SCB_MAINCLKUEN = SCB_MAINCLKUEN_DISABLE;      // Toggle update register once
  SCB_MAINCLKUEN = SCB_MAINCLKUEN_UPDATE;

  // Wait until the clock is updated
  while (!(SCB_MAINCLKUEN & SCB_MAINCLKUEN_UPDATE));
}

/**************************************************************************/
/*! 
    @brief Initialises the power management unit
*/
/**************************************************************************/
void pmuInit( void )
{
  /* Enable all clocks, even those turned off at power up. */
  SCB_PDRUNCFG &= ~(SCB_PDRUNCFG_WDTOSC_MASK | 
                    SCB_PDRUNCFG_SYSOSC_MASK | 
                    SCB_PDRUNCFG_ADC_MASK);
  return;
}

/**************************************************************************/
/*! 
    @brief Puts select peripherals in sleep mode.

    This function will put the device into sleep mode.  Most gpio pins
    can be used to wake the device up, but the pins must first be
    configured for this in pmuInit.

    @section Example
    @code 
    // Configure wakeup sources before going into sleep/deep-sleep.
    // By default, pin 0.1 is configured as wakeup source (falling edge)
    pmuInit();
  
    // Enter sleep mode
    pmuSleep();
    @endcode
*/
/**************************************************************************/
void pmuSleep()
{
  SCB_PDAWAKECFG = SCB_PDRUNCFG;
  __asm volatile ("WFI");
  return;
}

/**************************************************************************/
/*! 
    @brief  Turns off select peripherals and puts the device in deep-sleep
            mode.

    The device can be configured to wakeup from deep-sleep mode after a
    specified delay by supplying a non-zero value to the wakeupSeconds
    parameter.  This will configure CT32B0 to toggle pin 0.1 (CT32B0_MAT2)
    after x seconds, waking the device up.  The timer will be configured
    to run off the WDT OSC while in deep-sleep mode, meaning that WDTOSC
    should not be powered off (using the sleepCtrl parameter) when a
    wakeup delay is specified.

    The sleepCtrl parameter is used to indicate which peripherals should
    be put in sleep mode (see the SCB_PDSLEEPCFG register for details).
    
    @param[in]  sleepCtrl  
                The bits to set in the SCB_PDSLEEPCFG register.  This
                controls which peripherals will be put in sleep mode.
    @param[in]  wakeupSeconds
                The number of seconds to wait until the device will
                wakeup.  If you do not wish to wakeup after a specific
                delay, enter a value of 0.

    @code 
    uint32_t pmuRegVal;
  
    // Configure wakeup sources before going into sleep/deep-sleep
    // By default, pin 0.1 is configured as wakeup source
    pmuInit();
  
    // Put peripherals into sleep mode
    pmuRegVal = SCB_PDSLEEPCFG_IRCOUT_PD |
                SCB_PDSLEEPCFG_IRC_PD |
                SCB_PDSLEEPCFG_FLASH_PD |
                SCB_PDSLEEPCFG_BOD_PD |
                SCB_PDSLEEPCFG_ADC_PD |
                SCB_PDSLEEPCFG_SYSPLL_PD;
  
    // Enter deep sleep mode (wakeup after 5 seconds)
    pmuDeepSleep(pmuRegVal, 5);
    @endcode
*/
/**************************************************************************/
void pmuDeepSleep(uint32_t sleepCtrl, uint32_t wakeupSeconds)
{
  SCB_PDAWAKECFG = SCB_PDRUNCFG;
  sleepCtrl &= ~(1 << 9);               // MAIN_REGUL_PD
  sleepCtrl |= (1 << 11) | (1 << 12);   // LP_REGUL_PD
  SCB_PDSLEEPCFG = sleepCtrl;
  SCB_SCR |= SCB_SCR_SLEEPDEEP;

  /* Configure system to run from WDT and set TMR32B0 for wakeup          */
  if (wakeupSeconds > 0)
  {
    // Make sure WDTOSC isn't disabled in PDSLEEPCFG
    SCB_PDSLEEPCFG &= ~(SCB_PDSLEEPCFG_WDTOSC_PD);

    // Disable 32-bit timer 0 if currently in use
    TMR_TMR32B0TCR = TMR_TMR32B0TCR_COUNTERENABLE_DISABLED;

    // Disable internal pullup on 0.1
    gpioSetPullup(&IOCON_PIO0_1, gpioPullupMode_Inactive);

    // Reconfigure clock to run from WDTOSC
    pmuWDTClockInit();

    /* Enable the clock for CT32B0 */
    SCB_SYSAHBCLKCTRL |= (SCB_SYSAHBCLKCTRL_CT32B0);
  
    /* Configure 0.1 as Timer0_32 MAT2 */
    IOCON_PIO0_1 &= ~IOCON_PIO0_1_FUNC_MASK;
    IOCON_PIO0_1 |= IOCON_PIO0_1_FUNC_CT32B0_MAT2;

    /* Set appropriate timer delay */
    TMR_TMR32B0MR0 = PMU_WDTCLOCKSPEED_HZ * wakeupSeconds;
  
    /* Configure match control register to raise an interrupt and reset on MR0 */
    TMR_TMR32B0MCR |= (TMR_TMR32B0MCR_MR0_INT_ENABLED | TMR_TMR32B0MCR_MR0_RESET_ENABLED);
  
    /* Configure external match register to set 0.1 high on match */
    TMR_TMR32B0EMR &= ~(0xFF<<4);                   // Clear EMR config bits
    TMR_TMR32B0EMR |= TMR_TMR32B0EMR_EMC2_HIGH;     // Set MR2 (0.1) high on match

    /* Enable wakeup interrupt (P0.1..11 and P1.0 can be used) */
    //NVIC_EnableIRQ(WAKEUP0_IRQn);    // P0.0
    NVIC_EnableIRQ(WAKEUP1_IRQn);      // P0.1  (CT32B0_MAT2)
    //NVIC_EnableIRQ(WAKEUP2_IRQn);    // P0.2
    //NVIC_EnableIRQ(WAKEUP3_IRQn);    // P0.3
    //NVIC_EnableIRQ(WAKEUP4_IRQn);    // P0.4
    //NVIC_EnableIRQ(WAKEUP5_IRQn);    // P0.5
    //NVIC_EnableIRQ(WAKEUP6_IRQn);    // P0.6
    //NVIC_EnableIRQ(WAKEUP7_IRQn);    // P0.7
    //NVIC_EnableIRQ(WAKEUP8_IRQn);    // P0.8
    //NVIC_EnableIRQ(WAKEUP9_IRQn);    // P0.9
    //NVIC_EnableIRQ(WAKEUP10_IRQn);   // P0.10
    //NVIC_EnableIRQ(WAKEUP11_IRQn);   // P0.11 (CT32B0_MAT3)
    //NVIC_EnableIRQ(WAKEUP12_IRQn);   // P1.0

    /* Use RISING EDGE for wakeup detection. */
    SCB_STARTAPRP0 |= SCB_STARTAPRP0_APRPIO0_1;
  
    /* Clear all wakeup sources */ 
    SCB_STARTRSRP0CLR = SCB_STARTRSRP0CLR_MASK;

    /* Enable Port 0.1 as wakeup source. */
    SCB_STARTERP0 |= SCB_STARTERP0_ERPIO0_1;
  
    /* Start the timer */
    TMR_TMR32B0TCR = TMR_TMR32B0TCR_COUNTERENABLE_ENABLED;
  }

  // Send Wait For Interrupt command
  __asm volatile ("WFI");
  return;
}

/**************************************************************************/
/*! 
    @brief Puts the device in deep power-down mode.

    This function will configure the PMU control register and enter
    deep power-down mode.  Pre-determined values are stored in the four
    general-purpose registers (PMU_GPREG0..3), which can be used to persist
    any essential system settings while the device is in deep power-down
    mode, so long as 3.3V is still available.

    @warning    The only way to wake a device up from deep power-down mode
                is to set a low-level on P1.4.  If 3.3V power is lost, the
                values stored in the four general-purpose registers will
                also be lost.

    @section Example

    @code 
    #include "core/cpu/cpu.h"
    #include "core/pmu/pmu.h"

    int main(void)
    {
      cpuInit();
      pmuInit();

      // Enter power-down mode
      pmuPowerDown();

      while(1)
      {
        // Device was woken up by WAKEUP pin
      }
    }
    @endcode
*/
/**************************************************************************/
void pmuPowerDown( void )
{
  uint32_t regVal;

  if ( (PMU_PMUCTRL & ((0x1<<8) | (PMU_PMUCTRL_DPDFLAG))) != 0x0 )
  {
    /* Check sleep and deep power down bits. If sleep and/or
       deep power down mode are entered, clear the PCON bits. */
    regVal = PMU_PMUCTRL;
    regVal |= ((0x1<<8) | 
               (PMU_PMUCTRL_DPDEN_SLEEP) |
               (PMU_PMUCTRL_DPDFLAG));
    PMU_PMUCTRL = regVal;

    if ( (PMU_GPREG0 != 0x12345678)||(PMU_GPREG1 != 0x87654321)
       ||(PMU_GPREG2 != 0x56781234)||(PMU_GPREG3 != 0x43218765) )
    {
      while (1);
    }
  }
  else
  {
    /* If in neither sleep nor deep-sleep mode, enter deep power down mode. */
    PMU_GPREG0 = 0x12345678;
    PMU_GPREG1 = 0x87654321;
    PMU_GPREG2 = 0x56781234;
    PMU_GPREG3 = 0x43218765;
    SCB_SCR |= SCB_SCR_SLEEPDEEP;
    PMU_PMUCTRL = PMU_PMUCTRL_DPDEN_DEEPPOWERDOWN;
    __asm volatile ("WFI");
  }
  return;
}
