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

#if defined CFG_CHIBI
  #include <string.h>
  #include <stdlib.h>
  #include "drivers/chibi/chb.h"
  #include "drivers/chibi/chb_drvr.h"
  #include "core/uart/uart.h"
  static chb_rx_data_t rx_data;
#endif

/**************************************************************************/
/*! 
    Use Chibi as a wireless sniffer and write all captured frames
    to the SD card in libpcap format
  
    projectconfig.h settings:
    --------------------------------------------------
    CFG_CHIBI             -> Enabled
    CFG_CHIBI_PROMISCUOUS -> 1
    CFG_CHIBI_BUFFERSIZE  -> 1024   
*/
/**************************************************************************/
int main(void)
{
  // Configure cpu and mandatory peripherals
  systemInit();

  // Check if projectconfig.h is properly configured for this example
  #if !defined CFG_CHIBI
    #error "CFG_CHIBI must be enabled in projectconfig.h for this example"
  #endif
  #if CFG_CHIBI_PROMISCUOUS == 0
    #error "CFG_CHIBI_PROMISCUOUS must set to 1 in projectconfig.h for this example"
  #endif
  #if !defined CFG_PRINTF_UART
    #error "CFG_PRINTF_UART must be enabled in projectconfig.h for this example"
  #endif
  #if defined CFG_INTERFACE
    #error "CFG_INTERFACE must be disabled in projectconfig.h for this example"
  #endif

  #if defined CFG_CHIBI && CFG_CHIBI_PROMISCUOUS != 0
    // Get a reference to the Chibi peripheral control block
    chb_pcb_t *pcb = chb_get_pcb();
    
    // Wait for incoming frames and transmit the raw data over uart
    while(1)
    {
      // Check for incoming messages 
      while (pcb->data_rcv) 
      { 
        // get the length of the data
        rx_data.len = chb_read(&rx_data);
        // make sure the length is nonzero
        if (rx_data.len)
        {
          // Enable LED to indicate message reception 
          gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_ON); 

          // Send raw data to UART for processing on
          // the PC (requires WSBridge - www.freaklabs.org)
          uint8_t i;
          for (i=0; i<rx_data.len; i++)
          {
            // Send output to UART
            uartSendByte(rx_data.data[i]);
          }

          // Disable LED
          gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_OFF); 
        }
      }
    }
  #endif

  return 0;
}
