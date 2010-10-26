/**************************************************************************/
/*! 
    @file     projectconfig.h
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

#ifndef _PROJECTCONFIG_H_
#define _PROJECTCONFIG_H_

#include "lpc111x.h"
#include "sysdefs.h"
#include "drivers/chibi/chb_drvr.h"

/**************************************************************************

    This table tries to give an indication of which GPIO pins and 
    peripherals are used by the available drivers and SW examples.  Only
    dedicated GPIO pins available on the LPC1114 Reference Board are shown
    below.  Any unused peripheral blocks like I2C, SSP, ADC, etc., can
    also be used as GPIO if they are available.

                PORT 0    PORT 1    PORT 2               PORT 3
                =======   ======    =================    ======
                3 11      2 8 9     4 5 6 7 8 9 10 11    1 2 3

    SDCARD      X X       . . .     . . . . . . .  .     . . .
    PWM         . .       . . X     . . . . . . .  .     . . .
    STEPPER     . .       . . .     . . . . X X X  X     . . .
    CHIBI       . .       . . .     . . . . . . .  .     X X X
    ST7565      . .       . . .     X X X X X X .  .     . . .

                TIMERS                      SSP       ADC
                ======================      ===       ======
                16B0  16B1  32B0  32B1      0 1       1 2 6 7

    SDCARD      .     .     .     .         . X       . . . .
    PWM         .     X     .     .         . .       . . . .
    PMU [1]     .     .     X     .         . .       . . . .
    STEPPER     X     .     .     .         . .       . . . .
    CHIBI       .     .     .     .         X .       . . . .
    ST7565      .     .     .     .         . .       . . . .

    [1]  PMU uses 32-bit Timer 0 for SW wakeup from deep-sleep.  This timer
         can safely be used by other peripherals, but may need to be
         reconfigured when you wakeup from deep-sleep.

 **************************************************************************/


/*=========================================================================
    CORE CPU SETTINGS
    -----------------------------------------------------------------------

    CFG_CPU_CCLK    Value is for reference only.  'core/cpu/cpu.c' must
                    be modified to change the clock speed, but the value
                    should be indicated here since CFG_CPU_CCLK is used by
                    other peripherals to determine timing.

    -----------------------------------------------------------------------*/
    #define CFG_CPU_CCLK                (36000000)
/*=========================================================================*/


/*=========================================================================
    SYSTICK TIMER
    -----------------------------------------------------------------------

    CFG_SYSTICK_DELAY_IN_MS   The number of milliseconds between each tick
                              of the systick timer.

    -----------------------------------------------------------------------*/
    #define CFG_SYSTICK_DELAY_IN_MS     (1)
/*=========================================================================*/


/*=========================================================================
    UART
    -----------------------------------------------------------------------

    CFG_UART_BAUDRATE         The default UART speed.  This value is used 
                              when initialising UART, and should be a 
                              standard value like 57600, 9600, etc.
    CFG_UART_BUFSIZE          The length in bytes of the UART RX FIFO. This
                              will determine the maximum number of received
                              characters to store in memory.

    -----------------------------------------------------------------------*/
    #define CFG_UART_BAUDRATE           (115200)
    #define CFG_UART_BUFSIZE            (128)
/*=========================================================================*/


/*=========================================================================
    ON-BOARD LED
    -----------------------------------------------------------------------

    CFG_LED_PORT              The port for the on board LED
    CFG_LED_PIN               The pin for the on board LED
    CFG_LED_ON                The pin state to turn the LED on (0 = low, 1 = high)
    CFG_LED_OFF               The pin state to turn the LED off (0 = low, 1 = high)

    -----------------------------------------------------------------------*/
    #define CFG_LED_PORT                (3)
    #define CFG_LED_PIN                 (5)
    #define CFG_LED_ON                  (0)
    #define CFG_LED_OFF                 (1)
/*=========================================================================*/


/*=========================================================================
    MICRO-SD CARD
    -----------------------------------------------------------------------

    CFG_SDCARD                If this field is defined SD Card and Fat32
                              file system support will be included
    CFG_SDCARD_CDPORT         The card detect port number
    CFG_SDCARD_CDPIN          The card detect pin number

    NOTE: CFG_SDCARD =        ~7.2 KB Flash and 0.6 KB SRAM (-Os)

    DEPENDENCIES:             SDCARD requires the use of SSP1.
    -----------------------------------------------------------------------*/
    // #define CFG_SDCARD
    #define CFG_SDCARD_CDPORT           (0)
    #define CFG_SDCARD_CDPIN            (3)
    #define CFG_SDCARD_ENPORT           (0)
    #define CFG_SDCARD_ENPIN            (11)
/*=========================================================================*/


/*=========================================================================
    PRINTF REDIRECTION
    -----------------------------------------------------------------------

    CFG_PRINTF_UART           Will cause all printf statements to be 
                              redirected to UART
    CFG_PRINTF_CWDEBUG        Will cause all printf statements to be
                              redirected to the Crossworks
                              debug_printf statement (Crossworks only)
                              Warning: This is very slow!
    CFG_PRINTF_NEWLINE        This should be either "\r\n" for Windows or
                              "\n" for *nix

    Note: If no printf redirection definitions are present, all printf
    output will be ignored, though this will also save ~350 bytes flash.

    NOTE: PRINTF Support =    ~350 bytes Flash (-Os)
    -----------------------------------------------------------------------*/
    #define CFG_PRINTF_UART
    // #define CFG_PRINTF_CWDEBUG
    #define CFG_PRINTF_NEWLINE          "\r\n"
/*=========================================================================*/


/*=========================================================================
    COMMAND LINE INTERFACE
    -----------------------------------------------------------------------

    CFG_INTERFACE             If this field is defined the UART or USBCDC
                              based command-line interface will be included
    CFG_INTERFACE_MAXMSGSIZE  The maximum number of bytes to accept for an
                              incoming command
    CFG_INTERFACE_PROMPT      The command prompt to display at the start
                              of every new data entry line

    NOTE:                     The command-line interface will use either
                              USB-CDC or UART depending on whether
                              CFG_PRINTF_UART or CFG_PRINTF_USBCDC are 
                              selected.

    NOTE: CFG_INTERFACE =     ~6.0 KB Flash and 240 bytes SRAM (-Os), but
                              this varies with the number of commands
                              present
    -----------------------------------------------------------------------*/
    // #define CFG_INTERFACE
    #define CFG_INTERFACE_MAXMSGSIZE    (80)
    #define CFG_INTERFACE_PROMPT        "LPC1114 >> "
/*=========================================================================*/


/*=========================================================================
    PWM SETTINGS
    -----------------------------------------------------------------------

    CFG_PWM                     If this is defined, a basic PWM driver
                                will be included using 16-bit Timer 1 and
                                Pin 1.9 (MAT0) for the PWM output.  In
                                order to allow for a fixed number of
                                pulses to be generated, some PWM-specific
                                code is required in the 16-Bit Timer 1
                                ISR.  See "core/timer16/timer16.c" for
                                more information.
    CFG_PWM_DEFAULT_PULSEWIDTH  The default pulse width in ticks
    CFG_PWM_DEFAULT_DUTYCYCLE   The default duty cycle in percent

    DEPENDENCIES:               PWM output requires the use of 16-bit
                                timer 1 and pin 1.9 (CT16B1_MAT0).
    -----------------------------------------------------------------------*/
    // #define CFG_PWM
    #define CFG_PWM_DEFAULT_PULSEWIDTH  (CFG_CPU_CCLK / 1000)
    #define CFG_PWM_DEFAULT_DUTYCYCLE   (50)
/*=========================================================================*/


/*=========================================================================
    STEPPER MOTOR SETTINGS
    -----------------------------------------------------------------------

    CFG_STEPPER                 If this is defined, a simple bi-polar 
                                stepper motor will be included for common
                                H-bridge chips like the L293D or SN754410N

    DEPENDENCIES:               STEPPER requires the use of pins 2.8-11 and
                                16-bit timer 0.
    -----------------------------------------------------------------------*/
    // #define CFG_STEPPER
/*=========================================================================*/


/*=========================================================================
    EEPROM
    -----------------------------------------------------------------------

    CFG_I2CEEPROM             If defined, drivers for the onboard EEPROM
                              will be included during build
    CFG_I2CEEPROM_SIZE        The number of bytes available on the EEPROM

    -----------------------------------------------------------------------*/
    #define CFG_I2CEEPROM
    #define CFG_I2CEEPROM_SIZE          (4096)
/*=========================================================================*/


/*=========================================================================
    LM75B TEMPERATURE SENSOR
    -----------------------------------------------------------------------

    CFG_LM75B                 If defined, drivers for an optional LM75B
                              temperature sensor will be included during
                              build (requires external HW)

    -----------------------------------------------------------------------*/
    // #define CFG_LM75B
/*=========================================================================*/


/*=========================================================================
    CHIBI WIRELESS STACK
    -----------------------------------------------------------------------

    CFG_CHIBI                   If defined, the CHIBI wireless stack will be
                                included during build.  Requires external HW.
    CFG_CHIBI_MODE              The mode to use when receiving and transmitting
                                wireless data.  See chb_drvr.h for possible values
    CFG_CHIBI_POWER             The power level to use when transmitting.  See
                                chb_drvr.h for possible values
    CFG_CHIBI_CHANNEL           802.15.4 Channel (0 = 868MHz, 1-10 = 915MHz)
    CFG_CHIBI_PANID             16-bit PAN Identifier (ex.0x1234)
    CFG_CHIBI_BUFFERSIZE        The size of the message buffer in bytes
    CFG_CHIBI_EEPROM_IEEEADDR   Start location in EEPROM for the full IEEE
                                address of this node
    CFG_CHIBI_EEPROM_SHORTADDR  Start location in EEPROM for the short (16-bit)
                                address of this node

    NOTE: CFG_CHIBI =           ~4.0 KB Flash and 184 bytes SRAM* (-Os)
                                * 128 byte buffer

    DEPENDENCIES:               Chibi requires the use of SSP0, and pins 
                                3.1, 3.2, 3.3.  It also requires the
                                presence of CFG_I2CEEPROM.
    -----------------------------------------------------------------------*/
    // #define CFG_CHIBI
    #define CFG_CHIBI_MODE              (OQPSK_868MHZ)      // See chb_drvr.h for possible values
    #define CFG_CHIBI_POWER             (CHB_PWR_EU2_3DBM)  // See chb_drvr.h for possible values
    #define CFG_CHIBI_CHANNEL           (0)                 // 0 = 868-868.6 MHz, 1-10 = 915MHz
    #define CFG_CHIBI_PANID             (0x1234)
    #define CFG_CHIBI_BUFFERSIZE        (128)
    #define CFG_CHIBI_EEPROM_IEEEADDR   (uint16_t)(0x0000)
    #define CFG_CHIBI_EEPROM_SHORTADDR  (uint16_t)(0x0009)
/*=========================================================================*/


/*=========================================================================
    ST7565 128x64 Graphic LCD
    -----------------------------------------------------------------------

    CFG_ST7565                If defined, this will cause drivers for
                              the 128x64 pixel ST7565 LCD to be included

    Note:                     LPC1114 @ 36MHz and the ST7565 with the
                              backlight enabled consumes ~35mA

    DEPENDENCIES:             ST7565 requires the use of pins 2.4-9.
    -----------------------------------------------------------------------*/
    // #define CFG_ST7565
/*=========================================================================*/



// #####################
// Config error-checking
// #####################
#ifdef CFG_INTERFACE
  #ifndef CFG_PRINTF_UART
    #error "CFG_INTERFACE can not be used without CFG_PRINTF_UART"
  #endif
#endif

#ifdef CFG_CHIBI
  #if !defined CFG_I2CEEPROM
    #error "CFG_CHIBI requires CFG_I2CEEPROM to store and retrieve addresses"
  #endif
#endif

#ifdef CFG_ST7565
  #ifdef CFG_STEPPER
    #error "CFG_ST7565 and CFG_STEPPER can not be defined at the same time since they both use pins 2.8 and 2.9."
  #endif
#endif

#endif


