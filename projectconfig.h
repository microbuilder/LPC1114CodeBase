/**************************************************************************/
/*! 
    \file     projectconfig.h
    \author   K. Townsend (microBuilder.eu)
    \date     [date]
    \version  [version]
*/
/**************************************************************************/
#ifndef _PROJECTCONFIG_H_
#define _PROJECTCONFIG_H_

#include "lpc111x.h"
#include "sysdefs.h"

#define CFG_CPU_CCLK            (12000000)    // Ref. only.  Clock speed actually set in "core/cpu/cpu.c"

#define CFG_SYSTICK_DELAY_MS    (10)          // The number of ms between each tick of the systick timer

#define CFG_UART_BAUDRATE       (57600)       // Default UART speed

#define CFG_LED_PORT            (3)
#define CFG_LED_PIN             (5)
// #define CFG_LED_PORT            (0)
// #define CFG_LED_PIN             (3)

// #define CFG_CHIBI
// #define CFG_CHIBI_TRANSMITTER
#define CFG_CHIBI_RECEIVER


// Config error-handling
#ifdef CFG_CHIBI
  #if defined CFG_CHIBI_TRANSMITTER && defined CFG_CHIBI_RECEIVER
    #error "CFG_CHIBI_TRANSMITTER and CFG_CHIBI_RECEIVER can not both be defined at once"
  #endif
#endif

#endif
