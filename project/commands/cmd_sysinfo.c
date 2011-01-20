/**************************************************************************/
/*! 
    @file     cmd_sysinfo.c
    @author   K. Townsend (microBuilder.eu)

    @brief    Code to execute for cmd_sysinfo in the 'core/cmd'
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
#include "core/adc/adc.h"
#include "core/systick/systick.h"
#include "core/gpio/gpio.h"

#ifdef CFG_CHIBI
  #include "drivers/chibi/chb.h"
  #include "drivers/chibi/chb_drvr.h"
#endif

#ifdef CFG_LM75B
  #include "drivers/sensors/lm75b/lm75b.h"
#endif

/**************************************************************************/
/*! 
    'sysinfo' command handler
*/
/**************************************************************************/
void cmd_sysinfo(uint8_t argc, char **argv)
{
  printf("%-25s : %d.%d MHz %s", "System Clock", CFG_CPU_CCLK / 1000000, CFG_CPU_CCLK % 1000000, CFG_PRINTF_NEWLINE);
  printf("%-25s : %d.%d.%d %s", "Firmware", CFG_FIRMWARE_VERSION_MAJOR, CFG_FIRMWARE_VERSION_MINOR, CFG_FIRMWARE_VERSION_REVISION, CFG_PRINTF_NEWLINE);

  // Check the battery voltage
  #ifdef CFG_BAT
    gpioSetDir(CFG_BAT_ENPORT, CFG_BAT_ENPIN, gpioDirection_Output );   // Set voltage divider enable pin to output
    gpioSetValue(CFG_BAT_ENPORT, CFG_BAT_ENPIN, 1 );                    // Enable the voltage divider
    systickDelay(5);
    // Read ADC several times
    uint32_t i, c, ctotal;
    c = ctotal = 0;
    for (i = 0; i < 5; i++)
    {
      c = adcRead(CFG_BAT_ADC);
      ctotal += c;
    }
    // Get average of all readings
    uint32_t v = ctotal / 5;
    v = ((v * 1000) / 1024);                 
    v = (v * 3300) / 1000;                // Value in millivolts relative to supply voltage
    v = (v * CFG_BAT_MULTIPLIER) / 1000;  // Regular battery voltage in millivolts
    // Turn the voltage divider back off
    gpioSetValue(CFG_BAT_ENPORT, CFG_BAT_ENPIN, 1 );
    printf("%-25s : %u.%u V %s", "Supply Voltage", (unsigned int)(v / 1000), (unsigned int)(v % 1000), CFG_PRINTF_NEWLINE);
  #endif

  // Wireless Settings (if CFG_CHIBI enabled)
  #ifdef CFG_CHIBI
    chb_pcb_t *pcb = chb_get_pcb();
    printf("%-25s : %s %s", "RF Transceiver", "AT86RF212", CFG_PRINTF_NEWLINE);
    #if CFG_CHIBI_PROMISCUOUS == 1
      printf("%-25s : %s %s", "RF Receive Mode", "Promiscuous", CFG_PRINTF_NEWLINE);
    #else
      printf("%-25s : %s %s", "RF Receive Mode", "Normal", CFG_PRINTF_NEWLINE);
    #endif
    printf("%-25s : 0x%04X (%d) %s", "802.15.4 PAN ID", CFG_CHIBI_PANID, CFG_CHIBI_PANID, CFG_PRINTF_NEWLINE);
    printf("%-25s : 0x%04X (%d) %s", "802.15.4 Node Address", pcb->src_addr, pcb->src_addr, CFG_PRINTF_NEWLINE);
    printf("%-25s : %d %s", "802.15.4 Channel", CFG_CHIBI_CHANNEL, CFG_PRINTF_NEWLINE);
  #endif

  // CLI and buffer Settings
  #ifdef CFG_INTERFACE
    printf("%-25s : %d bytes %s", "Max CLI Command", CFG_INTERFACE_MAXMSGSIZE, CFG_PRINTF_NEWLINE);
  #endif

  // System Uptime (based on systick timer)
  uint32_t currentTick = systickGetTicks();
  uint32_t rollovers = systickGetRollovers();
  uint32_t secsActive = currentTick / (1000 / CFG_SYSTICK_DELAY_IN_MS);
  secsActive += rollovers * (0xFFFFFFFF / (1000 / CFG_SYSTICK_DELAY_IN_MS));
  printf("%-25s : %u s %s", "System Uptime", (unsigned int)secsActive, CFG_PRINTF_NEWLINE);

  // System Temperature (if LM75B Present)
  #ifdef CFG_LM75B
    int32_t temp = 0;
    lm75bGetTemperature(&temp);
    temp *= 125;
    printf("%-25s : %d.%d C %s", "Temperature", (int)(temp / 1000), (int)(temp % 1000), CFG_PRINTF_NEWLINE);
  #endif

  #ifdef CFG_SDCARD
    printf("%-25s : %s %s", "SD Card Present", gpioGetValue(CFG_SDCARD_CDPORT, CFG_SDCARD_CDPIN) ? "True" : "False", CFG_PRINTF_NEWLINE);
  #endif
}
