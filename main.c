#include <stdio.h>

#include "projectconfig.h"
#include "core/cpu/cpu.h"
#include "core/uart/uart.h"
#include "core/gpio/gpio.h"
#include "core/systick/systick.h"
#include "core/timer32/timer32.h"
#include "core/pmu/pmu.h"

#ifdef CFG_INTERFACE_UART
  #include "core/cmd/cmd.h"
#endif

#ifdef CFG_I2CEEPROM
  #include "drivers/eeprom/mcp24aa/mcp24aa.h"
#endif

#ifdef CFG_CHIBI
  #include "drivers/chibi/chb.h"
#endif

/**************************************************************************/
/*! 
    @brief Toggles the LED at the specified port and pin

    @param[in]  portNum
                GPIO port number
    @param[in]  pinNum
                GPIO pin number
*/
/**************************************************************************/
static void toggleLED(uint8_t portNum, uint8_t pinNum)
{
  if (gpioGetValue(portNum, pinNum))
  {
    // Set LED low
    gpioSetValue (portNum, pinNum, 0);
  }
  else
  {
    // Set LED high
    gpioSetValue (portNum, pinNum, 1);
  }
}

/**************************************************************************/
/*! 
    Configures the core system clock and sets up any mandatory
    peripherals like the systick timer, UART for printf, etc.

    This function should set the HW to the default state you wish to be
    in coming out of reset/startup, such as disabling or enabling LEDs,
    setting specific pin states, etc.
*/
/**************************************************************************/
static void systemInit()
{
  // Setup the cpu and core clock
  cpuInit();

  // Initialise the systick timer (delay set in projectconfig.h)
  systickInit(CFG_SYSTICK_DELAY_IN_MS);

  // Initialise UART with the default baud rate (set in projectconfig.h)
  uartInit(CFG_UART_BAUDRATE);

  // Note: Printf can now be used

  // Initialise GPIO
  gpioInit();

  // Initialise power management unit
  pmuInit();

  // Set LED pin as output and turn LED off
  gpioSetDir(CFG_LED_PORT, CFG_LED_PIN, 1);
  gpioSetPullup(&IOCON_PIO3_5, gpioPullupMode_Inactive);
  gpioSetValue(CFG_LED_PORT, CFG_LED_PIN, CFG_LED_OFF);

  // Initialise EEPROM (if requested)
  #ifdef CFG_I2CEEPROM
    mcp24aaInit();
  #endif

  // Initialise Chibi (if requested)
  #ifdef CFG_CHIBI
    // // Write addresses to EEPROM for the first time
    // uint16_t addr_short = 0x0000;
    // uint64_t addr_ieee =  0x0000000000000000;
    // mcp24aaWriteBuffer(CFG_CHIBI_EEPROM_SHORTADDR, (uint8_t *)&addr_short, 2);
    // mcp24aaWriteBuffer(CFG_CHIBI_EEPROM_IEEEADDR, (uint8_t *)&addr_ieee, 8);
    chb_init();
    chb_pcb_t *pcb = chb_get_pcb();
    printf("%-40s : 0x%04X%s", "Chibi Initialised", pcb->src_addr, CFG_INTERFACE_NEWLINE);
  #endif
}

int main (void)
{
  // Configure cpu and mandatory peripherals
  systemInit();

  // Start the command line (if requested)
  #ifdef CFG_INTERFACE
  printf("%sType 'help' for a list of available commands%s", CFG_INTERFACE_NEWLINE, CFG_INTERFACE_NEWLINE);
  cmdInit();
  #endif

  while (1)
  {
//    #ifdef CFG_CHIBI
//      #ifdef CFG_CHIBI_TRANSMITTER
//        i++;
//        sprintf(buf,"%ld",i);
//        gpioSetValue (2, 10, 0);
//        chb_write(0xFFFF, (uint8_t *)buf, 11);
//        gpioSetValue (2, 10, 1);
//        timer32Delay(0, TIMER32_DELAY_1S);
//      #endif
//      #ifdef CFG_CHIBI_RECEIVER
//        if (pcb->data_rcv)
//        {
//          rx_data.len = chb_read(&rx_data);
//          // Enable LED to indicate message reception (set low)
//          gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_ON);
//          // Output message to UART
//          printf("Message received from node %02X: %s (rssi=%d)%s", rx_data.src_addr, rx_data.data, pcb->ed, CFG_INTERFACE_NEWLINE);
//          // Disable LED (set high)
//          gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_OFF);
//          pcb->data_rcv = FALSE;
//        }
//      #endif
//    #endif

      #ifdef CFG_INTERFACE
        // Handle any incoming command line input
        cmdPoll();
      #endif

       // Toggle LED @ 1 Hz
       systickDelay(500);
       toggleLED(CFG_LED_PORT, CFG_LED_PIN);
  }
}

/**************************************************************************/
/*! 
    @brief Sends a single byte over UART.

    @param[in]  byte
                Byte value to send
*/
/**************************************************************************/
void __putchar(char c) 
{
  #ifdef CFG_INTERFACE_UART
    uartSendByte(c);
  #else
    // Send printf output to another endpoint
  #endif
}

/**************************************************************************/
/*! 
    @brief Sends a single byte over UART.

    @param[in]  byte
                Byte value to send
*/
/**************************************************************************/
int puts ( const char * str )
{
  while(*str++) __putchar(*str);
  return 0;
}
