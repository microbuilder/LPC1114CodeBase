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

#ifdef CFG_PRINTF_UART
  #include "core/uart/uart.h"
#endif

/**************************************************************************/
/*! 
    Sends a command to the TFT LCD, blocks until the command has
    finished executing on the LCD side, and returns any response
*/
/**************************************************************************/
byte_t* lcd(const byte_t* command)
{
  byte_t abtRx[256];
  size_t szRxLen;

  // Send the command to the LCD with '\n' (triggers cmd execution)
  printf("%s\n", command);

  // Wait for LCD busy pin to clear
  while (gpioGetValue(1, 4) == 1);

  // Check if there's a response on the uart buffer
  if (uartRxBufferReadArray(abtRx,&szRxLen))
  {
    // ToDo: scan until \r or \n
    // debug_printf(abtRx);
  }
  else
  {
    return "";
  }
}

/**************************************************************************/
/*! 
    This assumes that the LPC1343 TFTLCD board is hooked up to UART on
    the LPC1114, and that the LCD 'busy' pin is connected to pin 1.4.
*/
/**************************************************************************/
int main(void)
{
  // Configure cpu and mandatory peripherals
  systemInit();

  // Clear the UART buffer
  uartRxBufferInit();
  gpioSetDir(1, 4, 0);

  // Wait for LCD busy pin to clear (1 = active, 0 = ready)
  while (gpioGetValue(1, 4) == 1);

  lcd("clr24 255 255 255");
  lcd("text 10 10 0x0000 1 Message received from 0xBEEF");
  lcd("text 10 30 0x0000 1 RSSI:");
  lcd("progress 80 25 100 15 67 0x1234 0x123F");
  lcd("btn 10 130 220 35 0 Send Reply");
  lcd("text 10 180 0x0000 0 THIS");
  lcd("text 10 190 0x0000 0 is");
  lcd("text 10 200 0x0000 0 a");
  lcd("text 10 210 0x0000 0 TEST");
  lcd("text 10 220 0x0000 0 WITH");
  lcd("text 10 230 0x0000 0 multiple");
  lcd("text 10 240 0x0000 0 command lines");

  // Wait forever
  while(1)
  {
  }

  return 0;
}
