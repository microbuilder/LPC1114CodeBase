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

/*=========================================================================
    BOARD SELECTION

    Because several boards use this code library with sometimes slightly
    different pin configuration, you will need to specify which board you
    are using by enabling one of the following definitions. The code base
    will then try to configure itself accordingly for that board.
    -----------------------------------------------------------------------*/
    #define CFG_BRD_LPC1114_REFDESIGN
    // #define CFG_BRD_LPC1114_802154WIRELESS
/*=========================================================================*/


/**************************************************************************

    This table tries to give an indication of which GPIO pins and 
    peripherals are used by the available drivers and SW examples.  Only
    dedicated GPIO pins available on the LPC1114 Reference Board are shown
    below.  Any unused peripheral blocks like I2C, SSP, ADC, etc., can
    also be used as GPIO if they are available.

                PORT 0    PORT 1    PORT 2               PORT 3
                =======   ======    =================    ======
                3 11      2 8 9     4 5 6 7 8 9 10 11    1 2 3

    SDCARD      . .       . . .     X X . . . . .  .     . . .
    PWM         . .       . . X     . . . . . . .  .     . . .
    STEPPER     . .       . . .     . . . . X X X  X     . . .
    CHIBI       . .       . . .     . . . . . . .  .     X X X
    ST7565      . .       . . .     X X X X X X .  .     . . .
    SSD1306     . .       . . .     X X X . X X .  .     . . .
    INTERFACE   . .       . . .     . . . . . . .  .     . . .
    BATTERY     . X       . . .     . . . . . . .  X     . . .
    VREG [1]    . .       . . .     . . . . . . X  .     . . .

                TIMERS                      SSP       ADC [0]       UART
                ======================      ===       ===========   ====
                16B0  16B1  32B0  32B1      0 1       0 1 2 3 6 7   0

    SDCARD      .     .     .     .         . X       . . . . . .   .
    PWM         .     X     .     .         . .       . . . . . .   .
    PMU [2]     .     .     X     .         . .       . . . . . .   .
    STEPPER     X     .     .     .         . .       . . . . . .   .
    CHIBI       x     .     .     .         X .       . . . . . .   .
    ST7565      .     .     .     .         . .       . . . . . .   .
    SSD1306     .     .     .     .         . .       . . . . . .   .
    INTERFACE   .     .     .     .         . .       . . . . . .   x
    BATTERY     .     .     .     .         . .       X . . . . .   .

    [0]  Not all ADC pins are available on all boards:
         LPC1114 Reference Design:          AD0, AD1, AD6, AD7
         LPC1114 802.15.4 Wireless Board:   AD0, AD1, AD2, AD3
    [1]  Only relevant of the TPS780 is used.  The GPIO pin is used to
         switch from 3.3V to 2.2V depending on the MCUs operating mode.
    [2]  PMU uses 32-bit Timer 0 for SW wakeup from deep-sleep.  This timer
         can safely be used by other peripherals, but may need to be
         reconfigured when you wakeup from deep-sleep.

 **************************************************************************/


/*=========================================================================
    FIRMWARE VERSION SETTINGS
    -----------------------------------------------------------------------*/
    #define CFG_FIRMWARE_VERSION_MAJOR            (0)
    #define CFG_FIRMWARE_VERSION_MINOR            (6)
    #define CFG_FIRMWARE_VERSION_REVISION         (1)
/*=========================================================================*/


/*=========================================================================
    CORE CPU SETTINGS
    -----------------------------------------------------------------------

    CFG_CPU_CCLK    Value is for reference only.  'core/cpu/cpu.c' must
                    be modified to change the clock speed, but the value
                    should be indicated here since CFG_CPU_CCLK is used by
                    other peripherals to determine timing.

    Note:           At 36MHz 1 tick = ~27.777ns or 0.02777us
    -----------------------------------------------------------------------*/
    #ifdef CFG_BRD_LPC1114_REFDESIGN
      #define CFG_CPU_CCLK              (36000000)
    #endif

    #ifdef CFG_BRD_LPC1114_802154WIRELESS
      #define CFG_CPU_CCLK              (36000000)
    #endif
/*=========================================================================*/


/*=========================================================================
    VOLTAGE REGULATOR
    -----------------------------------------------------------------------

    CFG_VREG_VCC_MAIN     Output voltage of the regulator in millivolts
    CFG_VREG_ALT_PRESENT  0 if no alternative voltage is available, 1
                          if it is available (TPS780, etc.)
    CFG_VREG_ALT_VCC      Alternate output voltage in millivolts on the
                          vreg in sleep or low-power mode.  (Only relevant
                          for dual-output regulators like the TPS780, etc.)
    CFG_VREG_ALT_PORT     GPIO port to enable alternate VREG output
    CFG_VREG_ALT_PIN      GPIO pin to enable alternate VREG output
    CFG_VREG_ALT_REG32    IOCON Register for the alt. output enable pin

    -----------------------------------------------------------------------*/
    #ifdef CFG_BRD_LPC1114_REFDESIGN
      #define CFG_VREG_VCC_MAIN         (3300)    // 3.3V * 1000
      #define CFG_VREG_ALT_PRESENT      (0)
      #define CFG_VREG_ALT_PRESENT      (0)
      #define CFG_VREG_ALT_VCC          (3300)
      #define CFG_VREG_ALT_PORT         (2)
      #define CFG_VREG_ALT_PIN          (10)
      #define CFG_VREG_ALT_REG32        (IOCON_PIO2_10)
    #endif

    #ifdef CFG_BRD_LPC1114_802154WIRELESS
      #define CFG_VREG_VCC_MAIN         (3300)    // 3.3V * 1000
      #define CFG_VREG_ALT_PRESENT      (1)
      #define CFG_VREG_ALT_VCC          (2200)    // TPS780 = 3.3V + 2.2V
      #define CFG_VREG_ALT_PORT         (2)
      #define CFG_VREG_ALT_PIN          (10)
      #define CFG_VREG_ALT_REG32        (IOCON_PIO2_10)
    #endif
/*=========================================================================*/


/*=========================================================================
    BATTERY
    -----------------------------------------------------------------------

    CFG_BAT                   If this field is defined it indicates
                              that a user selectable voltage divider is
                              connected to the batter/power supply
    CFG_BAT_ENPORT            The port to enable to battery voltage divider
    CFG_BAT_ENPIN             The pin to enable the battery voltage divider
    CFG_BAT_ADC               The adc port where that the battery is connected
    CFG_BAT_SUPPLYVOLTAGE     The regulated supply voltage in millivolts
    CFG_BAT_MULTIPLIER        Multiplier to convert the adc result to
                              battery voltage - ((R1 + R2) / R2) * 1000

    Note:                     For an example of using this information to
                              determine the battery/power-supply voltage
                              level see 'projects/commands/cmd_sysinfo.c'

                              These settings are not relevant to all boards!
                              'tools/schematics/AT86RF212LPC1114_v1.6.pdf'
                              show how 'BAT' is meant to be connected/used
    -----------------------------------------------------------------------*/
    #ifdef CFG_BRD_LPC1114_REFDESIGN
      // #define CFG_BAT
      #define CFG_BAT_ENPORT              (2)
      #define CFG_BAT_ENPIN               (11)
      #define CFG_BAT_ENREG32             (IOCON_PIO2_11)
      #define CFG_BAT_ADC                 (0)
      #define CFG_BAT_MULTIPLIER          (0)
    #endif

    #ifdef CFG_BRD_LPC1114_802154WIRELESS
      #define CFG_BAT
      #define CFG_BAT_ENPORT              (2)
      #define CFG_BAT_ENPIN               (11)
      #define CFG_BAT_ENREG32             (IOCON_PIO2_11)
      #define CFG_BAT_ADC                 (0)     // AD0 = P0.11
      #define CFG_BAT_MULTIPLIER          (3127)  // ((10.0 + 4.7) / 4.7) * 1000
    #endif
/*=========================================================================*/


/*=========================================================================
    SYSTICK TIMER
    -----------------------------------------------------------------------

    CFG_SYSTICK_DELAY_IN_MS   The number of milliseconds between each tick
                              of the systick timer.
							  
    -----------------------------------------------------------------------*/
    #ifdef CFG_BRD_LPC1114_REFDESIGN
      #define CFG_SYSTICK_DELAY_IN_MS     (1)
    #endif

    #ifdef CFG_BRD_LPC1114_802154WIRELESS
      #define CFG_SYSTICK_DELAY_IN_MS     (1)
    #endif
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
    #ifdef CFG_BRD_LPC1114_REFDESIGN
      #define CFG_UART_BAUDRATE           (115200)
      #define CFG_UART_BUFSIZE            (128)
    #endif

    #ifdef CFG_BRD_LPC1114_802154WIRELESS
      #define CFG_UART_BAUDRATE           (115200)
      #define CFG_UART_BUFSIZE            (128)
    #endif
/*=========================================================================*/


/*=========================================================================
    ON-BOARD LED
    -----------------------------------------------------------------------

    CFG_LED_PORT              The port for the on board LED
    CFG_LED_PIN               The pin for the on board LED
    CFG_LED_ON                The pin state to turn the LED on (0 = low, 1 = high)
    CFG_LED_OFF               The pin state to turn the LED off (0 = low, 1 = high)

    -----------------------------------------------------------------------*/
    #ifdef CFG_BRD_LPC1114_REFDESIGN
      #define CFG_LED_PORT                (3)
      #define CFG_LED_PIN                 (5)
      #define CFG_LED_ON                  (0)
      #define CFG_LED_OFF                 (1)
    #endif

    #ifdef CFG_BRD_LPC1114_802154WIRELESS
      #define CFG_LED_PORT                (3)
      #define CFG_LED_PIN                 (5)
      #define CFG_LED_ON                  (0)
      #define CFG_LED_OFF                 (1)
    #endif
/*=========================================================================*/


/*=========================================================================
    MICRO-SD CARD
    -----------------------------------------------------------------------

    CFG_SDCARD                If this field is defined SD Card and Fat32
                              file system support will be included
    CFG_SDCARD_READONLY       If this is set to 1, all commands to
                              write to the SD card will be removed
                              saving some flash space (-Os).
    CFG_SDCARD_CDPORT         The card detect port number
    CFG_SDCARD_CDPIN          The card detect pin number
    CFG_SDCARD_CDREG32        IOCON Register for the CD pin
    CFG_SDCARD_ENPORT         The power enable port number
    CFG_SDCARD_ENPIN          The power enable pin number
    CFG_SDCARD_ENREG32        IOCON Register for the EN pin

    DEPENDENCIES:             SDCARD requires the use of SSP1.

    Note:                     These settings are not relevant to all boards!
                              'tools/schematics/AT86RF212LPC1114_v1.6.pdf'
                              show how SDCARD is meant to be connected/used
    -----------------------------------------------------------------------*/
    #ifdef CFG_BRD_LPC1114_REFDESIGN
      // #define CFG_SDCARD
      #define CFG_SDCARD_READONLY         (0)
      #define CFG_SDCARD_CDPORT           (2)
      #define CFG_SDCARD_CDPIN            (4)
      #define CFG_SDCARD_CDREG32          (IOCON_PIO2_4)
      #define CFG_SDCARD_ENPORT           (2)
      #define CFG_SDCARD_ENPIN            (5)
      #define CFG_SDCARD_ENREG32          (IOCON_PIO2_5)
    #endif

    #ifdef CFG_BRD_LPC1114_802154WIRELESS
      #define CFG_SDCARD
      #define CFG_SDCARD_READONLY         (0)   // Must be 0 or 1
      #define CFG_SDCARD_CDPORT           (2)
      #define CFG_SDCARD_CDPIN            (4)
      #define CFG_SDCARD_CDREG32          (IOCON_PIO2_4)
      #define CFG_SDCARD_ENPORT           (2)
      #define CFG_SDCARD_ENPIN            (5)
      #define CFG_SDCARD_ENREG32          (IOCON_PIO2_5)
    #endif
/*=========================================================================*/


/*=========================================================================
    PRINTF REDIRECTION
    -----------------------------------------------------------------------

    CFG_PRINTF_UART           Will cause all printf statements to be 
                              redirected to UART
    CFG_PRINTF_NEWLINE        This should be either "\r\n" for Windows or
                              "\n" for *nix

    Note: If no printf redirection definitions are present, all printf
    output will be ignored, though this will also save ~350 bytes flash.

    NOTE: PRINTF Support =    ~350 bytes Flash (-Os)
    -----------------------------------------------------------------------*/
    #ifdef CFG_BRD_LPC1114_REFDESIGN
      #define CFG_PRINTF_UART
      #define CFG_PRINTF_NEWLINE          "\r\n"
    #endif

    #ifdef CFG_BRD_LPC1114_802154WIRELESS
      #define CFG_PRINTF_UART
      #define CFG_PRINTF_NEWLINE          "\r\n"
    #endif
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
    -----------------------------------------------------------------------*/
    #ifdef CFG_BRD_LPC1114_REFDESIGN
      #define CFG_INTERFACE
      #define CFG_INTERFACE_MAXMSGSIZE    (256)
      #define CFG_INTERFACE_PROMPT        "LPC1114 >> "
    #endif

    #ifdef CFG_BRD_LPC1114_802154WIRELESS
      #define CFG_INTERFACE
      #define CFG_INTERFACE_MAXMSGSIZE    (256)
      #define CFG_INTERFACE_PROMPT        "LPC1114 >> "
    #endif
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
    #ifdef CFG_BRD_LPC1114_REFDESIGN
      // #define CFG_PWM
      #define CFG_PWM_DEFAULT_PULSEWIDTH  (CFG_CPU_CCLK / 1000)
      #define CFG_PWM_DEFAULT_DUTYCYCLE   (50)
    #endif

    #ifdef CFG_BRD_LPC1114_802154WIRELESS
      // #define CFG_PWM
      #define CFG_PWM_DEFAULT_PULSEWIDTH  (CFG_CPU_CCLK / 1000)
      #define CFG_PWM_DEFAULT_DUTYCYCLE   (50)
    #endif
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
    #ifdef CFG_BRD_LPC1114_REFDESIGN
      // #define CFG_STEPPER
    #endif

    #ifdef CFG_BRD_LPC1114_802154WIRELESS
      // #define CFG_STEPPER
    #endif
/*=========================================================================*/


/*=========================================================================
    EEPROM
    -----------------------------------------------------------------------

    CFG_I2CEEPROM             If defined, drivers for the onboard EEPROM
                              will be included during build
    CFG_I2CEEPROM_SIZE        The number of bytes available on the EEPROM

    -----------------------------------------------------------------------*/
    #ifdef CFG_BRD_LPC1114_REFDESIGN
      #define CFG_I2CEEPROM
      #define CFG_I2CEEPROM_SIZE          (3072)
    #endif

    #ifdef CFG_BRD_LPC1114_802154WIRELESS
      #define CFG_I2CEEPROM
      #define CFG_I2CEEPROM_SIZE          (3072)
    #endif
/*=========================================================================*/


/*=========================================================================
    EEPROM MEMORY MAP
    -----------------------------------------------------------------------
    EEPROM is used to persist certain user modifiable values to make
    sure that these changes remain in effect after a reset or hard
    power-down.  The addresses in EEPROM for these various system
    settings/values are defined below.  The first 256 bytes of EEPROM
    are reserved for this (0x0000..0x00FF).

    CFG_EEPROM_RESERVED       The last byte of reserved EEPROM memory

          EEPROM Address (0x0000..0x00FF)
          ===============================
          0 1 2 3 4 5 6 7 8 9 A B C D E F
    000x  x x x x x x x x . x x . . . . .   Chibi
    001x  . . . . . . . . . . . . . . . .
    002x  . . . . . . . . . . . . . . . .
    003x  . . . . . . . . . . . . . . . .
    004x  . . . . . . . . . . . . . . . .
    005x  . . . . . . . . . . . . . . . .
    006x  . . . . . . . . . . . . . . . .
    007x  . . . . . . . . . . . . . . . .
    008x  . . . . . . . . . . . . . . . .
    009x  . . . . . . . . . . . . . . . .
    00Ax  . . . . . . . . . . . . . . . .
    00Bx  . . . . . . . . . . . . . . . .
    00Cx  . . . . . . . . . . . . . . . .
    00Dx  . . . . . . . . . . . . . . . .
    00Ex  . . . . . . . . . . . . . . . .
    00Fx  . . . . . . . . . . . . . . . .

    -----------------------------------------------------------------------*/
    #ifdef CFG_BRD_LPC1114_REFDESIGN
      #define CFG_EEPROM_RESERVED                 (0x00FF)              // Protect first 256 bytes of memory
      #define CFG_EEPROM_CHIBI_IEEEADDR           (uint16_t)(0x0000)    // 8
      #define CFG_EEPROM_CHIBI_SHORTADDR          (uint16_t)(0x0009)    // 2
    #endif

    #ifdef CFG_BRD_LPC1114_802154WIRELESS
      #define CFG_EEPROM_RESERVED                 (0x00FF)              // Protect first 256 bytes of memory
      #define CFG_EEPROM_CHIBI_IEEEADDR           (uint16_t)(0x0000)    // 8
      #define CFG_EEPROM_CHIBI_SHORTADDR          (uint16_t)(0x0009)    // 2
    #endif
/*=========================================================================*/


/*=========================================================================
    LM75B TEMPERATURE SENSOR
    -----------------------------------------------------------------------

    CFG_LM75B                 If defined, drivers for an optional LM75B
                              temperature sensor will be included during
                              build (requires external HW)

    -----------------------------------------------------------------------*/
    #ifdef CFG_BRD_LPC1114_REFDESIGN
      // #define CFG_LM75B
    #endif

    #ifdef CFG_BRD_LPC1114_802154WIRELESS
      #define CFG_LM75B
    #endif
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
    CFG_CHIBI_PROMISCUOUS       Set to 1 to enabled promiscuous mode or
                                0 to disable it.  If promiscuous mode is
                                enabled be sure to set CFG_CHIBI_BUFFERSIZE
                                to an appropriately large value (ex. 1024)
    CFG_CHIBI_BUFFERSIZE        The size of the message buffer in bytes

    DEPENDENCIES:               Chibi requires the use of SSP0, 16-bit timer
                                0 and pins 3.1, 3.2, 3.3.  It also requires
                                the presence of CFG_I2CEEPROM.

    NOTE:                       These settings are not relevant to all boards!
                                'tools/schematics/AT86RF212LPC1114_v1.6.pdf'
                                show how 'CHIBI' is meant to be connected
    -----------------------------------------------------------------------*/
    #ifdef CFG_BRD_LPC1114_REFDESIGN
      // #define CFG_CHIBI
      #define CFG_CHIBI_MODE              (0)                 // OQPSK_868MHZ
      #define CFG_CHIBI_POWER             (0xE9)              // CHB_PWR_EU2_3DBM
      #define CFG_CHIBI_CHANNEL           (0)                 // 868-868.6 MHz
      #define CFG_CHIBI_PANID             (0x1234)
      #define CFG_CHIBI_PROMISCUOUS       (0)
      #define CFG_CHIBI_BUFFERSIZE        (128)
    #endif

    #ifdef CFG_BRD_LPC1114_802154WIRELESS
      #define CFG_CHIBI
      #define CFG_CHIBI_MODE              (0)                 // OQPSK_868MHZ
      #define CFG_CHIBI_POWER             (0xE9)              // CHB_PWR_EU2_3DBM
      #define CFG_CHIBI_CHANNEL           (0)                 // 868-868.6 MHz
      #define CFG_CHIBI_PANID             (0x1234)
      #define CFG_CHIBI_PROMISCUOUS       (0)
      #define CFG_CHIBI_BUFFERSIZE        (128)
    #endif
/*=========================================================================*/


/*=========================================================================
    128x64 Graphic LCDs
    -----------------------------------------------------------------------

    CFG_ST7565                If defined, this will cause drivers for
                              the 128x64 pixel ST7565 LCD to be included
    CFG_SSD1306               If defined, this will cause drivers for
                              the 128x64 pixel SSD1306 OLED display to be
                              included

    Note:                     LPC1114 @ 36MHz and the ST7565 with the
                              backlight enabled consumes ~35mA

    DEPENDENCIES:             ST7565 requires the use of pins 2.4-9.
    DEPENDENCIES:             SSD1306 requires the use of pins 2.4-9.
    -----------------------------------------------------------------------*/
    #ifdef CFG_BRD_LPC1114_REFDESIGN
      // #define CFG_ST7565
      // #define CFG_SSD1306
    #endif

    #ifdef CFG_BRD_LPC1114_802154WIRELESS
      // #define CFG_ST7565
      // #define CFG_SSD1306
    #endif
/*=========================================================================*/


/*=========================================================================
    RSA Encryption
    -----------------------------------------------------------------------

    CFG_RSA                     If defined, support for basic RSA
                                encryption will be included.
    CFG_RSA_BITS                Indicates the number of bits used for
                                RSA encryption keys.  To keep code size
                                reasonable, RSA encryption is currently
                                limited to using 64-bit or 32-bit numbers,
                                with 64-bit providing higher security, and
                                32-bit providing smaller encrypted text
                                size.  Please note that Printf can not be
                                used to display 64-bit values (%lld)!
    -----------------------------------------------------------------------*/
    #ifdef CFG_BRD_LPC1114_REFDESIGN
      // #define CFG_RSA
      #define CFG_RSA_BITS                  (32)
    #endif

    #ifdef CFG_BRD_LPC1114_802154WIRELESS
      // #define CFG_RSA
      #define CFG_RSA_BITS                  (32)
    #endif
/*=========================================================================*/



/*=========================================================================
  CONFIG FILE VALIDATION
  -------------------------------------------------------------------------
  Basic error checking to make sure that incompatible defines are not 
  enabled at the same time, etc.

  =========================================================================*/

#if !defined CFG_BRD_LPC1114_REFDESIGN && !defined CFG_BRD_LPC1114_802154WIRELESS
  #error "You must defined a target board (CFG_BRD_LPC1114_REFDESIGN or CFG_BRD_LPC1114_802154WIRELESS)"
#endif

#if defined CFG_BRD_LPC1114_REFDESIGN && defined CFG_BRD_LPC1114_802154WIRELESS
  #error "Only one target board can be defined at a time (CFG_BRD_LPC1114_REFDESIGN or CFG_BRD_LPC1114_802154WIRELESS)"
#endif

#ifdef CFG_INTERFACE
  #ifndef CFG_PRINTF_UART
    #error "CFG_INTERFACE can not be used without CFG_PRINTF_UART"
  #endif
#endif

#ifdef CFG_CHIBI
  #if !defined CFG_I2CEEPROM
    #error "CFG_CHIBI requires CFG_I2CEEPROM to store and retrieve addresses"
  #endif
  #ifdef CFG_STEPPER
    #error "CFG_CHIBI and CFG_STEPPER can not be defined at the same time since they both use CT16B0"
  #endif
  #if CFG_CHIBI_PROMISCUOUS != 0 && CFG_CHIBI_PROMISCUOUS != 1
    #error "CFG_CHIBI_PROMISCUOUS must be equal to either 1 or 0"
  #endif
#endif

#ifdef CFG_ST7565
  #ifdef CFG_STEPPER
    #error "CFG_ST7565 and CFG_STEPPER can not be defined at the same time since they both use pins 2.8 and 2.9"
  #endif
  #ifdef CFG_SSD1306
    #error "CFG_ST7565 and CFG_SSD1306 can not be defined at the same time"
  #endif
#endif

#ifdef CFG_RSA
  #if CFG_RSA_BITS != 64 && CFG_RSA_BITS != 32
    #error "CFG_RSA_BITS must be equal to either 32 or 64."
  #endif
#endif

#if CFG_VREG_ALT_PRESENT != 1 && CFG_VREG_ALT_PRESENT != 0
  #error "CFG_VREG_ALT_PRESENT must be equal to either 1 or 0"
#endif

#endif


