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

#ifdef CFG_LM75B
  #include "drivers/sensors/lm75b/lm75b.h"
#endif

#ifdef CFG_CHIBI
  #include "drivers/chibi/chb.h"
  static chb_rx_data_t rx_data;
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

  // Initialie LM75B (if requested)
  #ifdef CFG_LM75B
    lm75bInit();
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
  
  // Start the command line (if requested)
  #ifdef CFG_INTERFACE
    printf("%sType 'help' for a list of available commands%s", CFG_INTERFACE_NEWLINE, CFG_INTERFACE_NEWLINE);
    cmdInit();
  #endif  
}

int main (void)
{
  // Configure cpu and mandatory peripherals
  systemInit();

  while (1)
  {
    #ifdef CFG_INTERFACE
      // Handle any incoming command line input
      cmdPoll();
    #else
      // Toggle LED @ 1 Hz
      systickDelay(1000);
      if (gpioGetValue(CFG_LED_PORT, CFG_LED_PIN))  
        gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_ON);
      else 
        gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_OFF);
    #endif
  }
}

/**************************************************************************/
/*! 
    @brief Sends a single byte over UART.

    @param[in]  byte
                Byte value to send
*/
/**************************************************************************/
void __putchar(const char c)
{
  #ifdef CFG_INTERFACE_UART
    // Redirect printf to UART
    uartSendByte(c);
  #else
    // Redirect printf to another endpoint
  #endif
}

/**************************************************************************/
/*! 
    @brief Sends a single byte over UART.

    @param[in]  byte
                Byte value to send
*/
/**************************************************************************/
int puts(const char * str)
{
  while(*str) __putchar(*str++);
  return 0;
}

// ToDo: Cleanup

/*    #ifdef CFG_CHIBI
      chb_pcb_t *pcb = chb_get_pcb();

      // Send message over Chibi every 500mS
      // systickDelay(500 / CFG_SYSTICK_DELAY_IN_MS);
      // gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_ON);
      // char *text = "Test";
      // chb_write(0xFFFF, text, sizeof(text));
      // gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_OFF);

      // Check for incoming messages
      if (pcb->data_rcv)
      {
        rx_data.len = chb_read(&rx_data);
        // Enable LED to indicate message reception (set low)
        gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_ON);
        // Output message to UART
        printf("Message received from node %02X: %s (rssi=%d)%s", rx_data.src_addr, rx_data.data, pcb->ed, CFG_INTERFACE_NEWLINE);
        // Disable LED (set high)
        gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_OFF);
        pcb->data_rcv = FALSE;
      }
    #endif
*/
