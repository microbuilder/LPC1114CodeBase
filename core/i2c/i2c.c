/**************************************************************************/
/*! 
    @file     i2c.c
    @author   NXP, moidified by K. Townsend (microBuilder.eu)
    @date     22 March 2010
    @version  0.10

    @section DESCRIPTION

    Controls the I2C peripheral block.  For an example of how to use
    this i2c code, see the NXP sample software for the LPC1343 or the
    driver provided in this package for Microchip's I2C 24AA32AF
    EEPROM ("drivers/eeprom/mcp24aa").

    @warning  This code is based directly on the sample code provided by
              NXP and will be completely rewritten in an upcoming
              release of the LPC1343 Code Base.  There are a number of
              improvements that can be made to it in terms of organization
              and error handling, but since writing generic I2C code is a 
              non-trivial task, we've included the current code to allow
              I2C access in the short term. Please keep in mind that this
              entire code block will likely be replaced in the near future.
			  
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

#include "i2c.h"

volatile i2cState_t I2CMasterState = I2CSTATE_IDLE;
volatile i2cState_t I2CSlaveState = I2CSTATE_IDLE;

volatile i2cMode_t I2CMode;

volatile uint8_t  I2CMasterBuffer[I2C_BUFSIZE];
volatile uint8_t  I2CSlaveBuffer[I2C_BUFSIZE];
volatile uint32_t I2CCount = 0;
volatile uint32_t I2CReadLength;
volatile uint32_t I2CWriteLength;

volatile uint32_t RdIndex = 0;
volatile uint32_t WrIndex = 0;

/*****************************************************************************
** Function name:		I2C_IRQHandler
**
** Descriptions:		I2C interrupt handler, deal with master mode only.
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void I2C_IRQHandler(void) 
{
  uint8_t StatValue;

  /* this handler deals with master read and master write only */
  StatValue = I2C_I2CSTAT;

  switch ( StatValue )
  {    
    case I2CERR_STARTTX:
      /* A Start condition is issued. */
      WrIndex = 0;
      I2C_I2CDAT = I2CMasterBuffer[WrIndex++];
      I2C_I2CCONCLR = (I2C_I2CCONCLR_SIC | I2C_I2CCONCLR_STAC);
      I2CMasterState = I2CSTATE_STARTED;
      break;
    
    case I2CERR_REPEATEDSTARTTX:
      /* A repeated started is issued */
      RdIndex = 0;
      /* Send SLA with R bit set, */
      I2C_I2CDAT = I2CMasterBuffer[WrIndex++];
      I2C_I2CCONCLR = (I2C_I2CCONCLR_SIC | I2C_I2CCONCLR_STAC);
      I2CMasterState = I2CSTATE_RESTARTED;
      break;
    
    case I2CERR_SLAWTX_ACKRX:
      /* Regardless, it's a ACK */
      if ( I2CMasterState == I2CSTATE_STARTED )
      {
        I2C_I2CDAT = I2CMasterBuffer[WrIndex++];
        I2CMasterState = I2CSTATE_DATA_ACK;
      }
      I2C_I2CCONCLR = I2C_I2CCONCLR_SIC;
      break;
    
    case I2CERR_DATTX_ACKRX:
    case I2CERR_DATTX_NACKRX:
      /* Data byte has been transmitted, regardless ACK or NACK */
      if ( WrIndex < I2CWriteLength )
      {   
        I2C_I2CDAT = I2CMasterBuffer[WrIndex++]; /* this should be the last one */
        I2CMasterState = I2CSTATE_DATA_ACK;
      }
      else
      {
        if ( I2CReadLength != 0 )
        {
          I2C_I2CCONSET = I2C_I2CCONSET_STA;	/* Set Repeated-start flag */
          I2CMasterState = I2CSTATE_REPEATED_START;
        }
        else
        {
          I2CMasterState = I2CSTATE_DATA_NACK;
          I2C_I2CCONSET = I2C_I2CCONSET_STO;      /* Set Stop flag */
        }
      }
      I2C_I2CCONCLR = I2C_I2CCONCLR_SIC;
      break;
    
    case I2CERR_SLARTX_ACKRX:
      /* Master Receive, SLA_R has been sent */
      if ( I2CReadLength == 1 )
      {
        /* Will go to State 0x58 */
        I2C_I2CCONCLR = I2C_I2CCONCLR_AAC;	/* assert NACK after data is received */
      }
      else
      {
        /* Will go to State 0x50 */
        I2C_I2CCONSET = I2C_I2CCONSET_AA;	/* assert ACK after data is received */
      }
      I2C_I2CCONCLR = I2C_I2CCONCLR_SIC;
      break;
    
    case I2CERR_DATRX_ACKTX:	
      /* Data byte has been received, regardless following ACK or NACK */
      I2CSlaveBuffer[RdIndex++] = I2C_I2CDAT;
      if ( RdIndex < I2CReadLength )
      {   
        I2CMasterState = I2CSTATE_DATA_ACK;
        I2C_I2CCONSET = I2C_I2CCONSET_AA;	/* assert ACK after data is received */
      }
      else
      {
        I2CMasterState = I2CSTATE_DATA_NACK;
        I2C_I2CCONCLR = I2C_I2CCONCLR_AAC;	/* assert NACK on last byte */
      }
      I2C_I2CCONCLR = I2C_I2CCONCLR_SIC;
      break;
    
    case I2CERR_DATRX_NACKTX:
      I2CSlaveBuffer[RdIndex++] = I2C_I2CDAT;
      I2CMasterState = I2CSTATE_DATA_NACK;
      I2C_I2CCONSET = I2C_I2CCONSET_STO;	/* Set Stop flag */ 
      I2C_I2CCONCLR = I2C_I2CCONCLR_SIC;	/* Clear SI flag */
      break;
  
    case I2CERR_SLAWTX_NACKRX:		/* regardless, it's a NACK */
    case I2CERR_SLARTX_NACKRX:
      I2C_I2CCONCLR = I2C_I2CCONCLR_SIC;
      I2CMasterState = I2CSTATE_DATA_NACK;
      break;
    
    case I2CERR_ARBLOST:
      /* Arbitration lost, in this example, we don't deal with multiple master situation */
                                    
    default:
      I2C_I2CCONCLR = I2C_I2CCONCLR_SIC;	
      break;
  }
  return;
}

/*****************************************************************************
** Function name:		I2CStart
**
** Descriptions:		Create I2C start condition, a timeout
**				value is set if the I2C never gets started,
**				and timed out. It's a fatal error. 
**
** parameters:			None
** Returned value:		true or false, return false if timed out
** 
*****************************************************************************/
uint32_t i2cStart( void )
{
  uint32_t timeout = 0;
  uint32_t retVal = FALSE;
 
  /*--- Issue a start condition ---*/
  I2C_I2CCONSET = I2C_I2CCONSET_STA;	/* Set Start flag */
    
  /*--- Wait until START transmitted ---*/
  while( 1 )
  {
    if ( I2CMasterState == I2CSTATE_STARTED )
    {
      retVal = TRUE;
      break;	
    }
    if ( timeout >= I2C_MAX_TIMEOUT )
    {
      retVal = FALSE;
      break;
    }
    timeout++;
  }
  return( retVal );
}

/*****************************************************************************
** Function name:		I2CStop
**
** Descriptions:		Set the I2C stop condition, if the routine
**				never exit, it's a fatal bus error.
**
** parameters:			None
** Returned value:		true or never return
** 
*****************************************************************************/
uint32_t i2cStop( void )
{
  I2C_I2CCONSET = I2C_I2CCONSET_STO;      /* Set Stop flag */ 
  I2C_I2CCONCLR = I2C_I2CCONCLR_SIC;  /* Clear SI flag */ 
            
  /*--- Wait for STOP detected ---*/
  while( I2C_I2CCONSET & I2C_I2CCONSET_STO );
  return TRUE;
}

/*****************************************************************************
** Function name:		I2CInit
**
** Descriptions:		Initialize I2C controller
**
** parameters:			mode is I2CMODE_MASTER or I2CMODE_SLAVE
** Returned value:		true or false, return false if the I2C
**				interrupt handler was not installed correctly
** 
*****************************************************************************/
uint32_t i2cInit( i2cMode_t mode ) 
{
  /* It seems to be bit0 is for I2C, different from
  UM. To be retested along with SSP reset. SSP and I2C
  reset are overlapped, a known bug, for now, both SSP 
  and I2C use bit 0 for reset enable. Once the problem
  is fixed, change to "#if 1". */
#if 1
  SCB_PRESETCTRL |= (0x1<<1);
#else
  SCB_PRESETCTRL |= (0x1<<0);
#endif

  // Enable I2C clock
  SCB_SYSAHBCLKCTRL |= (SCB_SYSAHBCLKCTRL_I2C);

  // Configure pin 0.4 for SCL
  IOCON_PIO0_4 &= ~(IOCON_PIO0_4_FUNC_MASK | IOCON_PIO0_4_I2CMODE_MASK);
  IOCON_PIO0_4 |= (IOCON_PIO0_4_FUNC_I2CSCL);

  // Configure pin 0.5 for SDA
  IOCON_PIO0_5 &= ~(IOCON_PIO0_5_FUNC_MASK | IOCON_PIO0_5_I2CMODE_MASK);
  IOCON_PIO0_5 |= IOCON_PIO0_5_FUNC_I2CSDA;

  // Clear flags
  I2C_I2CCONCLR = I2C_I2CCONCLR_AAC | 
                  I2C_I2CCONCLR_SIC | 
                  I2C_I2CCONCLR_STAC | 
                  I2C_I2CCONCLR_I2ENC;

  // See p.128 for appropriate values for SCLL and SCLH
#if I2C_FAST_MODE_PLUS
  IOCON_PIO0_4 |= (IOCON_PIO0_4_I2CMODE_FASTPLUSI2C);
  IOCON_PIO0_5 |= (IOCON_PIO0_5_I2CMODE_FASTPLUSI2C);
  I2C_I2CSCLL   = I2C_SCLL_HS_SCLL;
  I2C_I2CSCLH   = I2C_SCLH_HS_SCLH;
#else
  I2C_I2CSCLL   = I2C_SCLL_SCLL;
  I2C_I2CSCLH   = I2C_SCLH_SCLH;
#endif

  if ( mode == I2CMODE_SLAVE )
  {
    I2C_I2CADR0 = I2C_SLAVEADDR;
  }    

  /* Enable the I2C Interrupt */
  NVIC_EnableIRQ(I2C_IRQn);
  I2C_I2CCONSET = I2C_I2CCONSET_I2EN;

  return( TRUE );
}

/*****************************************************************************
** Function name:		I2CEngine
**
** Descriptions:		The routine to complete a I2C transaction
**				from start to stop. All the intermitten
**				steps are handled in the interrupt handler.
**				Before this routine is called, the read
**				length, write length, I2C master buffer,
**				and I2C command fields need to be filled.
**				see i2cmst.c for more details. 
**
** parameters:			None
** Returned value:		true or false, return false only if the
**				start condition can never be generated and
**				timed out. 
** 
*****************************************************************************/
uint32_t i2cEngine( void ) 
{
  I2CMasterState = I2CSTATE_IDLE;
  RdIndex = 0;
  WrIndex = 0;
  if ( i2cStart() != TRUE )
  {
    i2cStop();
    return ( FALSE );
  }

  while ( 1 )
  {
    if ( I2CMasterState == I2CSTATE_DATA_NACK )
    {
      i2cStop();
      break;
    }
  }    
  return ( TRUE );      
}
