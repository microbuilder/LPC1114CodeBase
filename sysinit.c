/**************************************************************************/
/*! 
    @file     sysinit.c
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

#include <stdio.h>
#include <stdlib.h>

#ifdef CFG_PRINTF_CWDEBUG
  #include <cross_studio_io.h>
#endif

#include "sysinit.h"

#include "core/cpu/cpu.h"
#include "core/pmu/pmu.h"

#ifdef CFG_PRINTF_UART
  #include "core/uart/uart.h"
#endif

#ifdef CFG_INTERFACE
  #include "core/cmd/cmd.h"
#endif

#ifdef CFG_ST7565
  #include "drivers/lcd/bitmap/st7565/st7565.h"
#endif

#ifdef CFG_SSD1306
  #include "drivers/lcd/bitmap/ssd1306/ssd1306.h"
#endif

#ifdef CFG_LM75B
  #include "drivers/sensors/lm75b/lm75b.h"
#endif

#ifdef CFG_CHIBI
  #include "drivers/chibi/chb.h"
#endif

#ifdef CFG_SDCARD
  #include "core/timer32/timer32.h"
  #include "core/ssp/ssp.h"
  #include "drivers/fatfs/diskio.h"
  #include "drivers/fatfs/ff.h"

  DWORD get_fattime ()
  {
    // ToDo!
    return 0;
  }
#endif

#ifdef CFG_I2CEEPROM
  #include "drivers/eeprom/mcp24aa/mcp24aa.h"
#endif

/**************************************************************************/
/*! 
    Configures the core system clock and sets up any mandatory
    peripherals like the systick timer, UART for printf, etc.

    This function should set the HW to the default state you wish to be
    in coming out of reset/startup, such as disabling or enabling LEDs,
    setting specific pin states, etc.
*/
/**************************************************************************/
void systemInit()
{
  // Setup the cpu and core clock
  cpuInit();

  // Initialise the systick timer (delay set in projectconfig.h)
  systickInit((CFG_CPU_CCLK / 1000) * CFG_SYSTICK_DELAY_IN_MS);

  // Initialise GPIO
  gpioInit();

  #ifdef CFG_PRINTF_UART
    // Initialise UART with the default baud rate (set in projectconfig.h)
    uartInit(CFG_UART_BAUDRATE);
  #endif

  // Printf can now be used

  // Initialise power management unit
  pmuInit();

  // Set LED pin as output and turn LED off
  gpioSetDir(CFG_LED_PORT, CFG_LED_PIN, 1);
  gpioSetValue(CFG_LED_PORT, CFG_LED_PIN, CFG_LED_OFF);

  // Initialise the ST7565 128x64 pixel display
  #ifdef CFG_ST7565
    st7565Init();
    st7565ClearScreen();    // Clear the screen  
    st7565Backlight(1);     // Enable the backlight
  #endif

  #ifdef CFG_SSD1306
    ssd1306Init(SSD1306_SWITCHCAPVCC);
    ssd1306ClearScreen();   // Clear the screen  
  #endif

  // Initialise EEPROM
  #ifdef CFG_I2CEEPROM
    mcp24aaInit();
  #endif

  // Initialise Chibi
  #ifdef CFG_CHIBI
    // Write addresses to EEPROM for the first time if necessary
    // uint16_t addr_short = 0x1166;
    // uint64_t addr_ieee =  addr_short;
    // mcp24aaWriteBuffer(CFG_CHIBI_EEPROM_SHORTADDR, (uint8_t *)&addr_short, 2);
    // mcp24aaWriteBuffer(CFG_CHIBI_EEPROM_IEEEADDR, (uint8_t *)&addr_ieee, 8);
    chb_init();
  #endif

  // Initialise SD Card
  // Normaly this should be handled on demand later, but you may wish to
  // initialise the SD card here in some situations
  #ifdef CFG_SDCARD
    // Turn off SD card by default (saves power)
    gpioSetDir( CFG_SDCARD_ENPORT, CFG_SDCARD_ENPIN, gpioDirection_Output ); /* Set enable pin to output */
    gpioSetValue( CFG_SDCARD_ENPORT, CFG_SDCARD_ENPIN, 0 ); /* Disable card by setting ENPIN low */
    // DSTATUS stat;
    // stat = disk_initialize(0);
    // if (stat & STA_NOINIT) 
    // {
    //  // Not initialised
    // }
    // if (stat & STA_NODISK) 
    // {
    //  // No disk
    // }
    // if (stat == 0)
    // {
    //  // SD card sucessfully initialised
    // }
  #endif

  #ifdef CFG_LM75B
    // Turn on LM75B (and put it in sleep mode by default to save power)
    lm75bInit();
  #endif

  #ifdef CFG_BAT
    // Turn off battery voltage divider by default
    gpioSetDir(CFG_BAT_ENPORT, CFG_BAT_ENPIN, gpioDirection_Output );   // Set voltage divider enable pin to output
    gpioSetValue(CFG_BAT_ENPORT, CFG_BAT_ENPIN, 0 );                    // Disable the voltage divider by default
  #endif

  // Start the command line interface (if requested)
  #ifdef CFG_INTERFACE
    printf("%sType 'help' for a list of available commands%s", CFG_PRINTF_NEWLINE, CFG_PRINTF_NEWLINE);
    cmdInit();
  #endif
}

/**************************************************************************/
/*! 
    @brief Sends a single byte to a pre-determined peripheral (UART, etc.).

    @param[in]  byte
                Byte value to send
*/
/**************************************************************************/
void __putchar(const char c) 
{
  #ifdef CFG_PRINTF_UART
    // Send output to UART
    uartSendByte(c);
  #endif

  #ifdef CFG_PRINTF_CWDEBUG
    // Send output to the Crossworks debug interface
    debug_printf("%c", c);
  #endif
}

/**************************************************************************/
/*! 
    @brief Sends a string to a pre-determined end point (UART, etc.).

    @param[in]  str
                Text to send

    @note This function is only called when using the GCC-compiler
          in Codelite or running the Makefile manually.  This function
          will not be called when using the C library in Crossworks for
          ARM.
*/
/**************************************************************************/
int puts(const char * str)
{
  while(*str) __putchar(*str++);
  return 0;
}

