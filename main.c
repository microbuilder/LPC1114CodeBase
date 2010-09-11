/**************************************************************************/
/*! 
    @file     main.c
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

#include "sysinit.h"

#ifdef CFG_INTERFACE
  #include "core/cmd/cmd.h"
#endif

#ifdef CFG_CHIBI
  #include "drivers/chibi/chb.h"
  #include "drivers/chibi/chb_buf.h"
  static chb_rx_data_t rx_data; 
#endif

/**************************************************************************/
/*! 
    Main program entry point.  After reset, normal code execution will
    begin here.
*/
/**************************************************************************/
int main (void)
{
  // Configure cpu and mandatory peripherals
  systemInit();

  #ifdef CFG_CHIBI
    chb_pcb_t *pcb = chb_get_pcb(); 
  #endif

  while (1)
  {
    #ifdef CFG_INTERFACE 
      // Handle any incoming command line input 
      cmdPoll(); 
    #endif  

    #ifdef CFG_CHIBI
      // Check for incoming messages 
      if (pcb->data_rcv) 
      { 
        rx_data.len = chb_read(&rx_data); 
        // Enable LED to indicate message reception 
        gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_ON); 
        printf("Message received from node %04X: %s, len=%d, rssi=%d%s", rx_data.src_addr, rx_data.data, rx_data.len, pcb->ed, CFG_PRINTF_NEWLINE); 
        printf("Buffer Len: %d%s", (int)chb_buf_get_len(), CFG_PRINTF_NEWLINE);
        // Disable LED
        gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_OFF); 
        pcb->data_rcv = FALSE; 
      }

//      // Send global message every second 
//      char *text = "Test"; 
//      gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_ON); 
//      chb_write(0xFFFF, text, sizeof(text) + 1); 
//      gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_OFF);  
//  
//      // Delay for 500mS
//      systickDelay(500);
    #endif
    
//    #ifdef CFG_INTERFACE 
//      // Handle any incoming command line input 
//      cmdPoll(); 
//    #else 
//      // Toggle LED @ 1 Hz 
//      systickDelay(1000); 
//      if (gpioGetValue(CFG_LED_PORT, CFG_LED_PIN))   
//        gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_ON); 
//      else  
//        gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_OFF); 
//    #endif  
  }
}

