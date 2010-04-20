/**************************************************************************/
/*! 
    @file     gpio.c
    @author   K. Townsend (microBuilder.eu)
    @date     22 March 2010
    @version  0.10

    @section DESCRIPTION
	
    Controls the general purpose digital IO.

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

#include "gpio.h"

#ifdef CFG_CHIBI
#include "drivers/chibi/chb_drvr.h"
volatile uint32_t chibi_counter  = 0;
#endif

volatile uint32_t gpio0_counter = 0;
volatile uint32_t gpio1_counter = 0;
volatile uint32_t gpio2_counter = 0;
volatile uint32_t gpio3_counter = 0;
volatile uint32_t p0_1_counter  = 0;
volatile uint32_t p1_1_counter  = 0;
volatile uint32_t p2_1_counter  = 0;
volatile uint32_t p3_1_counter  = 0;

/**************************************************************************/
/*! 
    @brief IRQ Handler for GPIO port 0 (currently checks pin 0.1)
*/
/**************************************************************************/
void PIOINT0_IRQHandler(void)
{
  uint32_t regVal;

  gpio0_counter++;

  regVal = gpioIntStatus(0, 1);
  if (regVal)
  {
    p0_1_counter++;
    gpioIntClear(0, 1);
  }		
  return;
}

/**************************************************************************/
/*! 
    @brief IRQ Handler for GPIO port 1 (currently checks pin 1.1)
*/
/**************************************************************************/
void PIOINT1_IRQHandler(void)
{
  uint32_t regVal;

  gpio1_counter++;

  regVal = gpioIntStatus(1, 1);
  if ( regVal )
  {
    p1_1_counter++;
    gpioIntClear(1, 1);
  }

  return;
}

/**************************************************************************/
/*! 
    @brief IRQ Handler for GPIO port 2 (currently checks pin 2.1)
*/
/**************************************************************************/
void PIOINT2_IRQHandler(void)
{
  uint32_t regVal;

  gpio2_counter++;
  regVal = gpioIntStatus(2, 1);
  if ( regVal )
  {
    p2_1_counter++;
    gpioIntClear(2, 1);
  }		
  return;
}

/**************************************************************************/
/*! 
    @brief IRQ Handler for GPIO port 3 (currently checks pin 3.1)
*/
/**************************************************************************/
void PIOINT3_IRQHandler(void)
{
  uint32_t regVal;

 #ifdef CFG_CHIBI
  // Check for interrupt on 3.1
  regVal = gpioIntStatus(3, 1);
  if ( regVal )
  {
    chibi_counter++;
    chb_ISR_Handler();
    gpioIntClear(3, 1);
  }		
#else
  gpio3_counter++;
  regVal = gpioIntStatus(3, 1);
  if ( regVal )
  {
    p3_1_counter++;
    gpioIntClear(3, 1);
  }		
#endif

 return;
}

/**************************************************************************/
/*! 
    @brief Initialises GPIO and enables the GPIO interrupt
           handler for all GPIO ports.
*/
/**************************************************************************/
void gpioInit (void)
{
  /* Enable AHB clock to the GPIO domain. */
  SCB_SYSAHBCLKCTRL |= (SCB_SYSAHBCLKCTRL_GPIO);

  /* Set up NVIC when I/O pins are configured as external interrupts. */
  NVIC_EnableIRQ(EINT0_IRQn);
  NVIC_EnableIRQ(EINT1_IRQn);
  NVIC_EnableIRQ(EINT2_IRQn);
  NVIC_EnableIRQ(EINT3_IRQn);
  return;
}

/**************************************************************************/
/*! 
    @brief Sets the direction (input/output) for a specific port pin

    @param[in]  portNum
                The port number (0..3)
    @param[in]  bitPos
                The bit position (0..11)
    @param[in]  dir
                The pin direction (gpioDirection_Input or
                gpioDirection_Output)
*/
/**************************************************************************/
void gpioSetDir (uint32_t portNum, uint32_t bitPos, gpioDirection_t dir)
{
  switch (portNum)
  {
    case 0:
      if (gpioDirection_Output == dir)
      {
        GPIO_GPIO0DIR |= (1 << bitPos);
      }
      else
      {
        GPIO_GPIO0DIR &= ~(1 << bitPos);
      }
      break;
    case 1:
      if (gpioDirection_Output == dir)
      {
        GPIO_GPIO1DIR |= (1 << bitPos);
      }
      else
      {
        GPIO_GPIO1DIR &= ~(1 << bitPos);
      }
      break;
    case 2:
      if (gpioDirection_Output == dir)
      {
        GPIO_GPIO2DIR |= (1 << bitPos);
      }
      else
      {
        GPIO_GPIO2DIR &= ~(1 << bitPos);
      }
      break;
    case 3:
      if (gpioDirection_Output == dir)
      {
        GPIO_GPIO3DIR |= (1 << bitPos);
      }
      else
      {
        GPIO_GPIO3DIR &= ~(1 << bitPos);
      }
      break;
  }
}

/**************************************************************************/
/*! 
    @brief Gets the value for a specific port pin

    @param[in]  portNum
                The port number (0..3)
    @param[in]  bitPos
                The bit position (0..31)

    @return     The current value for the specified port pin (0..1)
*/
/**************************************************************************/
uint32_t gpioGetValue (uint32_t portNum, uint32_t bitPos)
{
  uint32_t value = 0;

  switch (portNum)
  {
    case 0:
      value = (GPIO_GPIO0DATA & (1 << bitPos)) ? 1 : 0;
      break;
    case 1:
      value = (GPIO_GPIO1DATA & (1 << bitPos)) ? 1 : 0;
      break;
    case 2:
      value = (GPIO_GPIO2DATA & (1 << bitPos)) ? 1 : 0;
      break;
    case 3:
      value = (GPIO_GPIO3DATA & (1 << bitPos)) ? 1 : 0;
      break;
    default:
      break;
  }

  return value;
}

/**************************************************************************/
/*! 
    @brief Sets the value for a specific port pin (only relevant when a
           pin is configured as output).

    @param[in]  portNum
                The port number (0..3)
    @param[in]  bitPos
                The bit position (0..31)
    @param[in]  bitValue
                The value to set for the specified bit (0..1).  0 will set
                the pin low and 1 will set the pin high.
*/
/**************************************************************************/
void gpioSetValue (uint32_t portNum, uint32_t bitPos, uint32_t bitVal)
{
  switch (portNum)
  {
    case 0:
      if (1 == bitVal)
      {
        GPIO_GPIO0DATA |= (1 << bitPos);
      }
      else
      {
        GPIO_GPIO0DATA &= ~(1 << bitPos);
      }
      break;
    case 1:
      if (1 == bitVal)
      {
        GPIO_GPIO1DATA |= (1 << bitPos);
      }
      else
      {
        GPIO_GPIO1DATA &= ~(1 << bitPos);
      }
      break;
    case 2:
      if (1 == bitVal)
      {
        GPIO_GPIO2DATA |= (1 << bitPos);
      }
      else
      {
        GPIO_GPIO2DATA &= ~(1 << bitPos);
      }
      break;
    case 3:
      if (1 == bitVal)
      {
        GPIO_GPIO3DATA |= (1 << bitPos);
      }
      else
      {
        GPIO_GPIO3DATA &= ~(1 << bitPos);
      }
      break;
    default:
      break;
  }
}

/**************************************************************************/
/*! 
    @brief Sets the interrupt sense, event, etc.

    @param[in]  portNum
                The port number (0..3)
    @param[in]  bitPos
                The bit position (0..31)
    @param[in]  sense
                Whether the interrupt should be configured as edge or level
                sensitive.
    @param[in]  edge
                Whether one edge or both trigger an interrupt.
    @param[in]  event
                Whether the rising or the falling edge (high or low)
                should be used to trigger the interrupt.

    @section Example

    @code
    // Initialise gpio
    gpioInit();
    // Set GPIO1.8 to input
    gpioSetDir(1, 8, gpioDirection_Input);
    // Disable the internal pullup/down resistor on P1.8
    gpioSetPullup (&IOCON_PIO1_8, gpioPullupMode_Inactive);
    // Setup an interrupt on GPIO1.8
    gpioSetInterrupt(1,                               // Port
                     8,                               // Pin
                     gpioInterruptSense_Edge,         // Edge/Level Sensitive
                     gpioInterruptEdge_Single,        // Single/Double Edge
                     gpioInterruptEvent_ActiveHigh);  // Rising/Falling
    // Enable the interrupt
    gpioIntEnable(1, 8);
    @endcode
*/
/**************************************************************************/
void gpioSetInterrupt (uint32_t portNum, uint32_t bitPos, gpioInterruptSense_t sense, gpioInterruptEdge_t edge, gpioInterruptEvent_t event)
{
  switch (portNum)
  {
    case 0:
      if (gpioInterruptSense_Edge)
      {
        GPIO_GPIO0IS &= ~(0x1<<bitPos);
        /* single or double only applies when sense is 0(edge trigger). */
        if  (gpioInterruptEdge_Single)
        {
          GPIO_GPIO0IBE &= ~(0x1<<bitPos);
        }
        else
        {
          GPIO_GPIO0IBE |= (0x1<<bitPos);
        }
      }
      else
      {
        GPIO_GPIO0IS |= (0x1<<bitPos);
      }
      if (gpioInterruptEvent_ActiveHigh)
      {
        GPIO_GPIO0IEV &= ~(0x1<<bitPos);
      }
      else
      {
        GPIO_GPIO0IEV |= (0x1<<bitPos);
      }
      break;
    case 1:
      if (gpioInterruptSense_Edge)
      {
        GPIO_GPIO1IS &= ~(0x1<<bitPos);
        /* single or double only applies when sense is 0(edge trigger). */
        if  (gpioInterruptEdge_Single)
        {
          GPIO_GPIO1IBE &= ~(0x1<<bitPos);
        }
        else
        {
          GPIO_GPIO1IBE |= (0x1<<bitPos);
        }
      }
      else
      {
        GPIO_GPIO1IS |= (0x1<<bitPos);
      }
      if (gpioInterruptEvent_ActiveHigh)
      {
        GPIO_GPIO1IEV &= ~(0x1<<bitPos);
      }
      else
      {
        GPIO_GPIO1IEV |= (0x1<<bitPos);  
      }
      break;
    case 2:
      if (gpioInterruptSense_Edge)
      {
        GPIO_GPIO2IS &= ~(0x1<<bitPos);
        /* single or double only applies when sense is 0(edge trigger). */
        if  (gpioInterruptEdge_Single)
        {
          GPIO_GPIO2IBE &= ~(0x1<<bitPos);
        }
        else
        {
          GPIO_GPIO2IBE |= (0x1<<bitPos);
        }
      }
      else
      {
        GPIO_GPIO2IS |= (0x1<<bitPos);
      }
      if (gpioInterruptEvent_ActiveHigh)
      {
        GPIO_GPIO2IEV &= ~(0x1<<bitPos);
      }
      else
      {
        GPIO_GPIO2IEV |= (0x1<<bitPos);  
      }
      break;
    case 3:
      if (gpioInterruptSense_Edge)
      {
        GPIO_GPIO3IS &= ~(0x1<<bitPos);
        /* single or double only applies when sense is 0(edge trigger). */
        if  (gpioInterruptEdge_Single)
        {
          GPIO_GPIO3IBE &= ~(0x1<<bitPos);
        }
        else
        {
          GPIO_GPIO3IBE |= (0x1<<bitPos);
        }
      }
      else
      {
        GPIO_GPIO3IS |= (0x1<<bitPos);
      }
      if (gpioInterruptEvent_ActiveHigh)
      {
        GPIO_GPIO3IEV &= ~(0x1<<bitPos);
      }
      else
      {
        GPIO_GPIO3IEV |= (0x1<<bitPos);	  
      }
      break;
    default:
      break;
  }
  return;
}

/**************************************************************************/
/*! 
    @brief Enables the interrupt mask for a specific port pin

    @param[in]  portNum
                The port number (0..3)
    @param[in]  bitPos
                The bit position (0..31)
*/
/**************************************************************************/
void gpioIntEnable (uint32_t portNum, uint32_t bitPos)
{
  switch (portNum)
  {
    case 0:
      GPIO_GPIO0IE |= (0x1<<bitPos);
      break;
    case 1:
      GPIO_GPIO1IE |= (0x1<<bitPos);
      break;
    case 2:
      GPIO_GPIO2IE |= (0x1<<bitPos);
      break;
    case 3:
      GPIO_GPIO3IE |= (0x1<<bitPos);
      break;
    default:
      break;
  }
  return;
}

/**************************************************************************/
/*! 
    @brief Disables the interrupt mask for a specific port pin

    @param[in]  portNum
                The port number (0..3)
    @param[in]  bitPos
                The bit position (0..31)
*/
/**************************************************************************/
void gpioIntDisable (uint32_t portNum, uint32_t bitPos)
{
  switch (portNum)
  {
    case 0:
      GPIO_GPIO0IE &= ~(0x1<<bitPos); 
      break;
    case 1:
      GPIO_GPIO1IE &= ~(0x1<<bitPos);	
      break;
    case 2:
      GPIO_GPIO2IE &= ~(0x1<<bitPos);	    
      break;
    case 3:
      GPIO_GPIO3IE &= ~(0x1<<bitPos);	    
      break;
    default:
      break;
  }
  return;
}

/**************************************************************************/
/*! 
    @brief Gets the interrupt status for a specific port pin

    @param[in]  portNum
                The port number (0..3)
    @param[in]  bitPos
                The bit position (0..31)

    @return     The interrupt status for the specified port pin (0..1)
*/
/**************************************************************************/
uint32_t gpioIntStatus (uint32_t portNum, uint32_t bitPos)
{
  uint32_t regVal = 0;

  switch (portNum)
  {
    case 0:
      if (GPIO_GPIO0MIS & (0x1<<bitPos))
      {
        regVal = 1;
      }
      break;
    case 1:
      if (GPIO_GPIO1MIS & (0x1<<bitPos))
      {
        regVal = 1;	
      }
      break;
    case 2:
      if (GPIO_GPIO2MIS & (0x1<<bitPos))
      {
        regVal = 1;
      }		
      break;
    case 3:
      if (GPIO_GPIO3MIS & (0x1<<bitPos))
      {
        regVal = 1;
      }
      break;
    default:
      break;
  }
  return ( regVal );
}

/**************************************************************************/
/*! 
    @brief Clears the interrupt for a port pin

    @param[in]  portNum
                The port number (0..3)
    @param[in]  bitPos
                The bit position (0..31)
*/
/**************************************************************************/
void gpioIntClear (uint32_t portNum, uint32_t bitPos)
{
  switch (portNum)
  {
    case 0:
      GPIO_GPIO0IC |= (0x1<<bitPos); 
    break;
    case 1:
      GPIO_GPIO1IC |= (0x1<<bitPos);	
    break;
    case 2:
      GPIO_GPIO2IC |= (0x1<<bitPos);	    
    break;
    case 3:
      GPIO_GPIO3IC |= (0x1<<bitPos);	    
    break;
    default:
      break;
  }
  return;
}

/**************************************************************************/
/*! 
    @brief Configures the internal pullup/down resistor for GPIO pins
           (only relevant for pins configured as inputs)

    @param[in]  ioconReg
                A pointer to the IOCON registry value corresponding to
                the pin you wish to change (for example: &IOCON_PIO2_0
                for GPIO pin 2.0).
    @param[in]  mode
                The 'mode' that the pin should be set to, which must be
                correspond to a value defined in gpioPullupMode_t
    
    @warning    By default, all GPIO pins have the internal pull-up
                resistor enabled.  This may cause unusual behaviour if
                care isn't taken to set the internal resistor to an
                appropriate state.

    @section Example

    @code
    // Initialise gpio
    gpioInit();
    // Set GPIO1.8 to input
    gpioSetDir(1, 8, gpioDirection_Input);
    // Disable the internal pullup/down resistor on P1.8
    gpioSetPullup(&IOCON_PIO1_8, gpioPullupMode_Inactive);
    @endcode
*/
/**************************************************************************/
void gpioSetPullup (volatile uint32_t *ioconReg, gpioPullupMode_t mode)
{
  // ToDo: Disable interrupts while we are doing this?

  *ioconReg &= ~(IOCON_COMMON_MODE_MASK);
  *ioconReg |= mode;

  // ToDo: Re-enable interrupts?
};
