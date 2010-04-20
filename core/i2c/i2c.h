/**************************************************************************/
/*! 
    @file     i2c.h
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

#ifndef _I2C_H_
#define _I2C_H_

#define I2C_FAST_MODE_PLUS      0

#define I2C_BUFSIZE             12
#define I2C_MAX_TIMEOUT         0xFF

#define I2C_SLAVEADDR           0xA0

#define I2C_SCLH_SCLH           0x00000180  /* I2C SCL Duty Cycle High Reg */
#define I2C_SCLL_SCLL           0x00000180  /* I2C SCL Duty Cycle Low Reg */
#define I2C_SCLH_HS_SCLH        0x00000030  /* Fast Plus I2C SCL Duty Cycle High Reg */
#define I2C_SCLL_HS_SCLL        0x00000030  /* Fast Plus I2C SCL Duty Cycle Low Reg */

#include "projectconfig.h"

typedef enum i2cMode_e
{
  I2CMODE_MASTER              = 0x01,
  I2CMODE_SLAVE               = 0x02
} 
i2cMode_t;

typedef enum i2cErr_e
{
  I2CERR_BUSERROR             = 0x00,
  I2CERR_STARTTX              = 0x08,
  I2CERR_REPEATEDSTARTTX      = 0x10,
  I2CERR_SLAWTX_ACKRX         = 0x18,
  I2CERR_SLAWTX_NACKRX        = 0x20,
  I2CERR_DATTX_ACKRX          = 0x28,
  I2CERR_DATTX_NACKRX         = 0x30,
  I2CERR_ARBLOST              = 0x38,
  I2CERR_SLARTX_ACKRX         = 0x40,
  I2CERR_SLARTX_NACKRX        = 0x48,
  I2CERR_DATRX_ACKTX          = 0x50,
  I2CERR_DATRX_NACKTX         = 0x58,
  I2CERR_NOINFO               = 0xf8
}
i2cErr_t;

typedef enum i2cState_e
{
  I2CSTATE_IDLE               = 0,
  I2CSTATE_STARTED,
  I2CSTATE_RESTARTED,
  I2CSTATE_REPEATED_START,
  I2CSTATE_DATA_ACK,
  I2CSTATE_DATA_NACK
}
i2cState_t;

extern void I2C_IRQHandler( void );
extern uint32_t i2cInit( i2cMode_t mode );
extern uint32_t i2cStart( void );
extern uint32_t i2cStop( void );
extern uint32_t i2cEngine( void );

#endif
