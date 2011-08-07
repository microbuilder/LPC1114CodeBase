/**************************************************************************/
/*! 
    @file     main.c
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
#include "projectconfig.h"
#include "sysinit.h"

#include "core/gpio/gpio.h"
#include "core/pmu/pmu.h"

#if defined CFG_CHIBI
  #include <string.h>
  #include <stdlib.h>
  #include "drivers/chibi/chb.h"
  #include "drivers/chibi/chb_drvr.h"
#endif

#ifdef CFG_LM75B
  #include "drivers/sensors/lm75b/lm75b.h"
#endif

/**************************************************************************/
/*! 
    Broadcast a basic message every 250 milliseconds
  
    projectconfig.h settings:
    --------------------------------------------------
    CFG_CHIBI             -> Enabled
    CFG_CHIBI_PROMISCUOUS -> 0
    CFG_CHIBI_BUFFERSIZE  -> 128
*/
/**************************************************************************/
int main(void)
{
  // Configure cpu and mandatory peripherals
  systemInit();

  // Make sure that projectconfig.h is properly configured for this example
  #if !defined CFG_BRD_LPC1114_802154WIRELESS
    #error "Warning: This example (probably) only works with the LPC1114 802.15.4 Wireless board"
  #endif
  #if !defined CFG_CHIBI
    #error "CFG_CHIBI must be enabled in projectconfig.h for this example"
  #endif
  #if !defined CFG_LM75B
    #error "CFG_LM75B must be enabled in projectconfig.h for this example"
  #endif
  #if CFG_CHIBI_PROMISCUOUS != 0
    #error "CFG_CHIBI_PROMISCUOUS must be set to 0 in projectconfig.h for this example"
  #endif

  char text[128];
  int32_t temp;

  // Get a reference to the wireless peripheral control block
  chb_pcb_t *pcb = chb_get_pcb();

  // Send the temperature every 5 seconds and going into deep sleep between messages
  while(1)
  {
    // Get the current temperature (in 0.125°C units)
    lm75bGetTemperature(&temp);    
    // Multiply value by 125 for fixed-point math (0.125°C per unit)
    temp *= 125;
    // Use modulus operator to display decimal value
    sprintf(text, "Current Temp: %d.%d C", (int)(temp / 1000), (int)(temp % 1000));
    // Enable LED
    gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_ON); 
    chb_write(0xFFFF, text, strlen(text) + 1);
    // Disable LED
    gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_OFF); 
    // Deep sleep for 5 seconds
    pmuDeepSleep(5);
  }

  return 0;
}
