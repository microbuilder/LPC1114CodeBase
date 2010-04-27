#include <stdio.h>
#include <stdlib.h>

#include "projectconfig.h"
#include "core/cpu/cpu.h"
#include "core/uart/uart.h"
#include "core/gpio/gpio.h"
#include "core/systick/systick.h"
#include "core/timer32/timer32.h"
#include "core/pmu/pmu.h"

#ifdef CFG_CHIBI
#include "drivers/chibi/chb.h"
static chb_rx_data_t rx_data;
#endif

static void toggleLED(uint8_t portNum, uint8_t pinNum)
{
  if (gpioGetValue(portNum, pinNum))
  {
    // Enable LED (set low)
    gpioSetValue (portNum, pinNum, 0);
  }
  else
  {
    // Disable LED (set high)
    gpioSetValue (portNum, pinNum, 1);
  }
}

int main (void)
{
  cpuInit();
  systickInit(CFG_SYSTICK_DELAY_MS);
  uartInit(CFG_UART_BAUDRATE);

  printf("LPC1114 initialised @ 12MHz\n");
  printf("UART enabled at %d\r\n", CFG_UART_BAUDRATE);
  printf("SysTick timer set with %d mS delay\r\n", CFG_SYSTICK_DELAY_MS);

  // Set LED pin as output and turn off
  gpioInit();
  gpioSetDir(CFG_LED_PORT, CFG_LED_PIN, 1);
  gpioSetValue(CFG_LED_PORT, CFG_LED_PIN, 1);

  // Initialise 32-bit timer 0 with default delay 
  printf("Initialising 32-bit Timer 0 with 100 uS delay...\r\n");
  timer32Init(0, TIMER32_DEFAULTINTERVAL);
  timer32Enable(0);

  // Initialise power management unit
  printf("Initialising power-management unit...\r\n");
  pmuInit();

  // Initialise Chibi (AT86RF212)
  #ifdef CFG_CHIBI
  printf("Initialising Chibi (868MHz)...\n");
  pcb_t *pcb = chb_get_pcb();
  chb_init();
  printf("Done...\n");
  #endif

//  // Wait 10 second before going into deep sleep
//  printf("10 second delay before deep sleep...\r\n");
//  timer32Delay(0, TIMER32_DELAY_1S * 10);
//
//  // Put peripherals into sleep mode
//  uint32_t pmuRegVal = SCB_PDSLEEPCFG_IRCOUT_PD |
//    SCB_PDSLEEPCFG_IRC_PD |
//    SCB_PDSLEEPCFG_FLASH_PD |
//    SCB_PDSLEEPCFG_BOD_PD |
//    SCB_PDSLEEPCFG_ADC_PD |
//    SCB_PDSLEEPCFG_SYSPLL_PD | 
//    SCB_PDSLEEPCFG_SYSOSC_PD;
//
//  // If the wakeup timer is not used, WDTOSC can also be stopped (saves ~2uA)
//  // pmuRegVal |= SCB_PDSLEEPCFG_WDTOSC_PD;
//
//  // Enter deep sleep mode (wakeup after 5 seconds)
//  printf("Entering deep sleep (wakeup after 10 seconds)...\r\n");
//  pmuDeepSleep(pmuRegVal, 10);
//  printf("Wakeup successful...\r\n");

  while (1)
  {
    #ifdef CFG_CHIBI
      #ifdef CFG_CHIBI_TRANSMITTER
        i++;
        sprintf(buf,"%ld",i);
        gpioSetValue (2, 10, 0);
        chb_write(0xFFFF, (uint8_t *)buf, 11);
        gpioSetValue (2, 10, 1);
        timer32Delay(0, TIMER32_DELAY_1S);
      #endif
      #ifdef CFG_CHIBI_RECEIVER
        if (pcb->data_rcv)
        {
          rx_data.len = chb_read(&rx_data);
          // Enable LED to indicate message reception (set low)
          gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, 0);
          // Output message to UART
          printf("Message received from node %02X: %s (rssi=%d)\n", rx_data.src_addr, rx_data.data, pcb->ed);
          // Disable LED (set high)
          gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, 1);
          pcb->data_rcv = FALSE;
        }
      #endif
    #else
      // Toggle LED @ 1 Hz
      timer32Delay(0, TIMER32_DELAY_1S * 1);
      toggleLED(CFG_LED_PORT, CFG_LED_PIN);  
    #endif
  }
}

/**************************************************************************/
/*! 
    Redirect printf output to UART0
*/
/**************************************************************************/
void __putchar(char c) 
{
  uartSendByte(c);
}

int puts ( const char * str )
{
  while(*str++) __putchar(*str);
  return 0;
}
