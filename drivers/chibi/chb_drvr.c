/*******************************************************************
    Copyright (C) 2009 FreakLabs
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
    3. Neither the name of the the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
    OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.

    Originally written by Christopher Wang aka Akiba.
    Please post support questions to the FreakLabs forum.

*******************************************************************/
#include <stdio.h>
#include "chb.h"
#include "chb_drvr.h"
#include "chb_buf.h"
#include "chb_spi.h"
#include "chb_eeprom.h"

#include "core/timer32/timer32.h"

/**************************************************************************/
/*!

*/
/**************************************************************************/
static U8 chb_get_state()
{
    return chb_reg_read(TRX_STATUS) & 0x1f;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
static U8 chb_get_status()
{
    return chb_reg_read(TRX_STATE) >> CHB_TRAC_STATUS_POS;
}

/**************************************************************************/
/*!
    Cause a blocking delay for x microseconds
*/
/**************************************************************************/
static void chb_delay_us(U16 usec)
{
  U8 ticks = CFG_CPU_CCLK / 1000000;
  do
  {
    do 
    {
      __asm volatile("nop");
    } while (--ticks);
  } while (--usec);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_reset()
{
    CHB_RST_DISABLE();
    CHB_SLPTR_DISABLE();

    // wait a bit while transceiver wakes up
    chb_delay_us(TIME_P_ON_TO_CLKM_AVAIL);

    // reset the device
    CHB_RST_ENABLE();
    chb_delay_us(TIME_RST_PULSE_WIDTH);
    CHB_RST_DISABLE();

    // check that we have the part number that we're expecting
    while (1)
    {
        // if you're stuck in this loop, that means that you're not reading
        // the version and part number register correctly. possible that version number
        // changes. if so, update version num in header file
        if ((chb_reg_read(VERSION_NUM) == CHB_AT86RF212_VER_NUM) && (chb_reg_read(PART_NUM) == CHB_AT86RF212_PART_NUM)) 
        {
            break;
        }
    }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
U8 chb_reg_read(U8 addr)
{
    U8 val = 0;

    /* Add the register read command to the register address. */
    addr |= 0x80;

    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE();

    /*Send Register address and read register content.*/
    val = chb_xfer_byte(addr);
    val = chb_xfer_byte(val);

    CHB_SPI_DISABLE();
    CHB_LEAVE_CRIT();

    return val;
}

/**************************************************************************/
/*! 
 
*/
/**************************************************************************/
U16 chb_reg_read16(U8 addr)
{
    U8 i;
    U16 val = 0;

    for (i=0; i<2; i++)
    {
        addr |= chb_reg_read(addr + i) << (8 * i);
    }
    return val;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_reg_write(U8 addr, U8 val)
{
    U8 dummy; 

    /* Add the Register Write command to the address. */
    addr |= 0xC0;

    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE();

    /*Send Register address and write register content.*/
    dummy = chb_xfer_byte(addr);
    dummy = chb_xfer_byte(val);

    CHB_SPI_DISABLE();
    CHB_LEAVE_CRIT();
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_reg_write16(U8 addr, U16 val)
{
    U8 i;

    for (i=0; i<2; i++)
    {
        chb_reg_write(addr + i, val >> (8 * i));
    }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_reg_write64(U8 addr, U8 *val)
{
    U8 i;

    for (i=0; i<8; i++)
    {
        chb_reg_write(addr + i, *(val + i));
    }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_reg_read_mod_write(U8 addr, U8 val, U8 mask)
{
    U8 tmp;

    tmp = chb_reg_read(addr);
    val &= mask;                // mask off stray bits from val
    tmp &= ~mask;               // mask off bits in reg val
    tmp |= val;                 // copy val into reg val
    chb_reg_write(addr, tmp);   // write back to reg
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_frame_write(U8 *hdr, U8 hdr_len, U8 *data, U8 data_len)
{
    U8 i, dummy;

    // dont allow transmission longer than max frame size
    if ((hdr_len + data_len) > 127)
    {
        return;
    }

    // initiate spi transaction
    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE(); 

    // send fifo write command
    dummy = chb_xfer_byte(CHB_SPI_CMD_FW);

    // write hdr contents to fifo
    for (i=0; i<hdr_len; i++)
    {
        dummy = chb_xfer_byte(*hdr++);
    }

    // write data contents to fifo
    for (i=0; i<data_len; i++)
    {
        dummy = chb_xfer_byte(*data++);
    }

    // terminate spi transaction
    CHB_SPI_DISABLE(); 
    CHB_LEAVE_CRIT();
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
static void chb_frame_read()
{
    U8 i, len, data;

    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE();

    /*Send frame read command and read the length.*/
    chb_xfer_byte(CHB_SPI_CMD_FR);
    len = chb_xfer_byte(0);

    /*Check for correct frame length.*/

    // TODO: CHECK FOR THE CRC VALID BIT TO QUALIFY THE FRAME
    // check the length of the frame to make sure the incoming frame
    // doesn't overflow the buffer
    if ((len >= CHB_MIN_FRAME_LENGTH) && (len <= CHB_MAX_FRAME_LENGTH))
    {
        chb_buf_write(len);
        
        for (i=0; i<len; i++)
        {
            data = chb_xfer_byte(0);
            chb_buf_write(data);
        }
    }

    CHB_SPI_DISABLE();
    CHB_LEAVE_CRIT();
}

/**************************************************************************/
/*!
    Read directly from the SRAM on the radio. This is only used for debugging
    purposes.
*/
/**************************************************************************/
#ifdef CHB_DEBUG
void chb_sram_read(U8 addr, U8 len, U8 *data)
{
    U8 i, dummy;

    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE();

    /*Send SRAM read command.*/
    dummy = chb_xfer_byte(CHB_SPI_CMD_SR);

    /*Send address where to start reading.*/
    dummy = chb_xfer_byte(addr);

    for (i=0; i<len; i++)
    {
        *data++ = chb_xfer_byte(0);
    }

    CHB_SPI_DISABLE();
    CHB_LEAVE_CRIT();
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_sram_write(U8 addr, U8 len, U8 *data)
{    
    U8 i, dummy;

    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE();

    /*Send SRAM write command.*/
    dummy = chb_xfer_byte(CHB_SPI_CMD_SW);

    /*Send address where to start writing to.*/
    dummy = chb_xfer_byte(addr);

    for (i=0; i<len; i++)
    {
        dummy = chb_xfer_byte(*data++);
    }

    CHB_SPI_DISABLE();
    CHB_LEAVE_CRIT();
}
#endif

/**************************************************************************/
/*!
    Set the channel mode, BPSK, OQPSK, etc...
*/
/**************************************************************************/
void chb_set_mode(U8 mode)
{
    switch (mode)
    {
    case OQPSK_868MHZ:
        chb_reg_read_mod_write(TRX_CTRL_2, 0x08, 0x3f);                 // 802.15.4-2006, channel page 2, channel 0 (868 MHz, Europe)
        chb_reg_read_mod_write(RF_CTRL_0, CHB_OQPSK_TX_OFFSET, 0x3);    // this is according to table 7-16 in at86rf212 datasheet
        break;
    case OQPSK_915MHZ:
        chb_reg_read_mod_write(TRX_CTRL_2, 0x0c, 0x3f);                 // 802.15.4-2006, channel page 2, channels 1-10 (915 MHz, US)
        chb_reg_read_mod_write(RF_CTRL_0, CHB_OQPSK_TX_OFFSET, 0x3);    // this is according to table 7-16 in at86rf212 datasheet
        break;
    case OQPSK_780MHZ:
        chb_reg_read_mod_write(TRX_CTRL_2, 0x1c, 0x3f);                 // 802.15.4-2006, channel page 5, channel 0-3 (780 MHz, China)
        chb_reg_read_mod_write(RF_CTRL_0, CHB_OQPSK_TX_OFFSET, 0x3);    // this is according to table 7-16 in at86rf212 datasheet
        break;
    case BPSK40_915MHZ:
        chb_reg_read_mod_write(TRX_CTRL_2, 0x00, 0x3f);                 // 802.15.4-2006, BPSK, 40 kbps
        chb_reg_read_mod_write(RF_CTRL_0, CHB_BPSK_TX_OFFSET, 0x3);     // this is according to table 7-16 in at86rf212 datasheet
        break;
    case BPSK20_868MHZ:
        chb_reg_read_mod_write(TRX_CTRL_2, 0x00, 0x3f);                 // 802.15.4-2006, BPSK, 20 kbps
        chb_reg_read_mod_write(RF_CTRL_0, CHB_BPSK_TX_OFFSET, 0x3);     // this is according to table 7-16 in at86rf212 datasheet
        break;
    }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
U8 chb_set_channel(U8 channel)
{
    U8 state;
    
#if (CHB_CHINA == 1)

    // this if for China only which uses a 780 MHz frequency band
    if ((chb_reg_read(TRX_CTRL2) & 0x3f) != 0x1c)
    {
        chb_reg_read_mod_write(TRX_CTRL2, 0x1c, 0x3f);
    }
    
    if (channel > 3)
    {
        channel = 0;
    }

    channel = (channel << 1) + 11;      

    chb_reg_read_mod_write(CC_CTRL_1, 0x4, 0x7);                // set 769 MHz base frequency for China
    chb_reg_write(CC_CTRL_0, channel);                          // set the center frequency for the channel

#else
    //if (channel == 0)
    //{
    //    // Channel 0 is for European use only. make sure we are using channel page 2,
    //    // channel 0 settings for 100 kbps
    //    if ((chb_reg_read(TRX_CTRL_2) & 0x3f) != 0x08)
    //    {
    //        chb_reg_read_mod_write(TRX_CTRL_2, 0x08, 0x3f);
    //    }
    //}
    //else if (channel > 10)
    //{
    //    // if the channel is out of bounds for page 2, then default to channel 1 and 
    //    // assume we're on the US frequency of 915 MHz
    //    channel = 1;
    //    if ((chb_reg_read(TRX_CTRL_2) & 0x3f) != 0x0c)
    //    {
    //        chb_reg_read_mod_write(TRX_CTRL_2, 0x0c, 0x3f);
    //    }
    //}
    //else
    //{
    //    // Channels 1-10 are for US frequencies of 915 MHz
    //    if ((chb_reg_read(TRX_CTRL_2) & 0x3f) != 0x0c)
    //    {
    //        chb_reg_read_mod_write(TRX_CTRL_2, 0x0c, 0x3f);
    //    }
    //}
        
    chb_reg_read_mod_write(PHY_CC_CCA, channel, 0x1f); 
#endif

    // add a delay to allow the PLL to lock if in active mode.
    state = chb_get_state();
    if ((state == RX_ON) || (state == PLL_ON))
    {
        chb_delay_us(TIME_PLL_LOCK_TIME);
    }

    return ((chb_reg_read(PHY_CC_CCA) & 0x1f) == channel) ? RADIO_SUCCESS : RADIO_TIMED_OUT;
}

/**************************************************************************/
/*!
    Set the power level
*/
/**************************************************************************/
void chb_set_pwr(U8 val)
{
    chb_reg_write(PHY_TX_PWR, val);
}

/**************************************************************************/
/*!
    Set the TX/RX state machine state. Some manual manipulation is required 
    for certain operations. Check the datasheet for more details on the state 
    machine and manipulations.
*/
/**************************************************************************/
U8 chb_set_state(U8 state)
{
    U8 curr_state, delay;

    // if we're sleeping then don't allow transition
    if (gpioGetValue(CHB_SLPTRPORT, CHB_SLPTRPIN))
    {
        return RADIO_WRONG_STATE;
    }

    // if we're in a transition state, wait for the state to become stable
    curr_state = chb_get_state();
    if ((curr_state == BUSY_TX_ARET) || (curr_state == BUSY_RX_AACK) || (curr_state == BUSY_RX) || (curr_state == BUSY_TX))
    {
        while (chb_get_state() == curr_state);
    }

    // At this point it is clear that the requested new_state is:
    // TRX_OFF, RX_ON, PLL_ON, RX_AACK_ON or TX_ARET_ON.
    // we need to handle some special cases before we transition to the new state
    switch (state)
    {
    case TRX_OFF:
        /* Go to TRX_OFF from any state. */
        CHB_SLPTR_DISABLE();
        chb_reg_read_mod_write(TRX_STATE, CMD_FORCE_TRX_OFF, 0x1f);
        chb_delay_us(TIME_ALL_STATES_TRX_OFF);
        break;

    case TX_ARET_ON:
        if (curr_state == RX_AACK_ON)
        {
            /* First do intermediate state transition to PLL_ON, then to TX_ARET_ON. */
            chb_reg_read_mod_write(TRX_STATE, CMD_PLL_ON, 0x1f);
            chb_delay_us(TIME_RX_ON_PLL_ON);
        }
        break;

    case RX_AACK_ON:
        if (curr_state == TX_ARET_ON)
        {
            /* First do intermediate state transition to RX_ON, then to RX_AACK_ON. */
            chb_reg_read_mod_write(TRX_STATE, CMD_RX_ON, 0x1f);
            chb_delay_us(TIME_PLL_ON_RX_ON);
        }
        break;
    }

    /* Now we're okay to transition to any new state. */
    chb_reg_read_mod_write(TRX_STATE, state, 0x1f);

    /* When the PLL is active most states can be reached in 1us. However, from */
    /* TRX_OFF the PLL needs time to activate. */
    delay = (curr_state == TRX_OFF) ? TIME_TRX_OFF_PLL_ON : TIME_RX_ON_PLL_ON;
    chb_delay_us(delay);

    if (chb_get_state() == state)
    {
        return RADIO_SUCCESS;
    }
    return RADIO_TIMED_OUT;
}

/**************************************************************************/
/*! 
 
*/
/**************************************************************************/
void chb_set_ieee_addr(U8 *addr)
{
    chb_eeprom_write(CHB_EEPROM_IEEE_ADDR, addr, 8); 
    chb_reg_write64(IEEE_ADDR_0, addr); 
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_get_ieee_addr(U8 *addr)
{
    chb_eeprom_read(CHB_EEPROM_IEEE_ADDR, addr, 8);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_set_short_addr(U16 addr)
{
    U8 *addr_ptr = (U8 *)&addr;
    pcb_t *pcb = chb_get_pcb();

    chb_eeprom_write(CHB_EEPROM_SHORT_ADDR, addr_ptr, 2);
    chb_reg_write16(SHORT_ADDR_0, addr);
    pcb->src_addr = addr;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
U16 chb_get_short_addr()
{
    U8 addr[2];

    chb_eeprom_read(CHB_EEPROM_SHORT_ADDR, addr, 2);
    return *(U16 *)addr;
}

/**************************************************************************/
/*!
    Load the data into the fifo, initiate a transmission attempt,
    and return the status of the transmission attempt.
*/
/**************************************************************************/
U8 chb_tx(U8 *hdr, U8 *data, U8 len)
{
    U8 state = chb_get_state();
    pcb_t *pcb = chb_get_pcb();

    if ((state == BUSY_TX) || (state == BUSY_TX_ARET))
    {
        return RADIO_WRONG_STATE;
    }

    // TODO: check why we need to transition to the off state before we go to tx_aret_on
    chb_set_state(TRX_OFF);
    chb_set_state(TX_ARET_ON);

    // TODO: try and start the frame transmission by writing TX_START command instead of toggling
    // sleep pin...i just feel like it's kind of weird...

    // write frame to buffer. first write header into buffer (add 1 for len byte), then data. 
    chb_frame_write(hdr, CHB_HDR_SZ + 1, data, len);

    // TEST - check data in buffer
    //{
    //    U8 i, len, tmp[30];
    //    
    //    len = 1 + CHB_HDR_SZ + len;
    //    chb_sram_read(0, len, tmp);
    //    for (i=0; i<len; i++)
    //    {
    //        printf("%02X ", tmp[i]);
    //    }
    //    printf("\n");
    //    state = chb_get_state();
    //    printf("State = %02X.\n", state);
    //}
    //TEST

    //Do frame transmission. Toggle the SLP_TR pin to initiate the frame transmission.
    CHB_SLPTR_ENABLE();
    CHB_SLPTR_DISABLE();

    // wait for the transmission to end, signalled by the TRX END flag
    while (!pcb->tx_end);
    pcb->tx_end = false;

    // check the status of the transmission
    return chb_get_status();
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
static void chb_radio_init()
{
    U8 ieee_addr[8];

    // reset chip
    chb_reset();

    // disable intps while we config the radio
    chb_reg_write(IRQ_MASK, 0);

    // force transceiver off while we configure the intps
    chb_reg_read_mod_write(TRX_STATE, CMD_FORCE_TRX_OFF, 0x1F);

    // make sure the transceiver is in the off state before proceeding
    while ((chb_reg_read(TRX_STATUS) & 0x1f) != TRX_OFF);

    // set radio cfg parameters
    // **note** uncomment if these will be set to something other than default
    //chb_reg_read_mod_write(XAH_CTRL_0, CHB_MAX_FRAME_RETRIES << CHB_MAX_FRAME_RETRIES_POS, 0xF << CHB_MAX_FRAME_RETRIES_POS);
    //chb_reg_read_mod_write(XAH_CTRL_0, CHB_MAX_CSMA_RETRIES << CHB_MAX_CSMA_RETIRES_POS, 0x7 << CHB_MAX_CSMA_RETIRES_POS);
    //chb_reg_read_mod_write(CSMA_SEED_1, CHB_CSMA_SEED1 << CHB_CSMA_SEED1_POS, 0x7 << CHB_CSMA_SEED1_POS);
    //chb_ret_write(CSMA_SEED0, CHB_CSMA_SEED0);     
    //chb_reg_read_mod_write(PHY_CC_CCA, CHB_CCA_MODE << CHB_CCA_MODE_POS,0x3 << CHB_CCA_MODE_POS);
    //chb_reg_write(CCA_THRES, CHB_CCA_ED_THRES);

    // set frame version that we'll accept
    chb_reg_read_mod_write(CSMA_SEED_1, CHB_FRM_VER << CHB_FVN_POS, 3 << CHB_FVN_POS);

    // set interrupt mask
    // re-enable intps while we config the radio
    chb_reg_write(IRQ_MASK, (1<<IRQ_RX_START) | (1<<IRQ_TRX_END));

    // set autocrc mode
    chb_reg_read_mod_write(TRX_CTRL_1, 1 << CHB_AUTO_CRC_POS, 1 << CHB_AUTO_CRC_POS);

    // set up default phy modulation and data rate - OQPSK, 100 kbps, 868 MHz
    chb_set_mode(OQPSK_868MHZ);

    // Set power to 3db
    chb_set_pwr(CHB_PWR_EU2_3DBM);

    chb_set_channel(CHB_CHANNEL);

    // set fsm state
    // put trx in rx auto ack mode
    chb_set_state(RX_AACK_ON);

    // set pan ID
    chb_reg_write16(PAN_ID_0, CHB_PAN_ID);

    // set short addr
    // NOTE: Possibly get this from EEPROM
    chb_reg_write16(SHORT_ADDR_0, chb_get_short_addr());

    // set long addr
    // NOTE: Possibly get this from EEPROM
    chb_get_ieee_addr(ieee_addr);
    chb_reg_write64(IEEE_ADDR_0, ieee_addr);

    // set interrupt/gpio pin to input
    gpioSetDir (CHB_EINTPORT, CHB_EINTPIN, 0);

    // set internal resistor on EINT pin to inactive
    gpioSetPullup (&CHB_EINTPIN_IOCONREG, gpioPullupMode_Inactive);

    // configure pin for interrupt
    gpioSetInterrupt (CHB_EINTPORT,
                      CHB_EINTPIN,
                      gpioInterruptSense_Edge,        // Edge-sensitive
                      gpioInterruptEdge_Single,       // Single edge
                      gpioInterruptEvent_ActiveHigh); // High triggers interrupt

    // enable interrupt
    gpioIntEnable (CHB_EINTPORT,
                   CHB_EINTPIN); 

    while (chb_get_state() != RX_AACK_ON);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_drvr_init()
{
    // ToDo: Make sure gpioInit has been called
    // ToDo: Make sure CT32B0 has been initialised and enabled

    // config SPI for at86rf230 access
    chb_spi_init();

    // Set sleep and reset as output
    gpioSetDir(CHB_SLPTRPORT, CHB_SLPTRPIN, 1);
    gpioSetDir(CHB_RSTPORT, CHB_RSTPIN, 1);

    // configure IOs
    gpioSetValue(CHB_SLPTRPORT, CHB_SLPTRPIN, 1);  // Set sleep high
    gpioSetValue(CHB_RSTPORT, CHB_RSTPIN, 1);      // Set reset high

    // Set internal resistors
    gpioSetPullup (&CHB_SLPTRPIN_IOCONREG, gpioPullupMode_Inactive);
    gpioSetPullup (&CHB_RSTPIN_IOCONREG, gpioPullupMode_Inactive);

    // config radio
    chb_radio_init();
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
U8 chb_radio_sleep(void)
{
  uint32_t timeout = 0;

  // Set mode to TRX_OFF
  while ( timeout < 10 )
  {
    uint8_t status = chb_set_state(TRX_OFF);
    if (status == RADIO_SUCCESS)
    {
      break;
    }
    timeout++;
  }

  if ( timeout == 10 )
  {
    return 1;
  }
  else
  {
    // Set SLP_TR high to enter sleep mode (stops after 35 clock cycles)
    CHB_SLPTR_ENABLE();
    return 0;
  }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_ISR_Handler (void)
{
    U8 dummy, state, intp_src = 0;
    pcb_t *pcb = chb_get_pcb();

    CHB_ENTER_CRIT();

    /*Read Interrupt source.*/
    CHB_SPI_ENABLE();   

    /*Send Register address and read register content.*/
    dummy = chb_xfer_byte(IRQ_STATUS | CHB_SPI_CMD_RR);
    intp_src = chb_xfer_byte(0);

    CHB_SPI_DISABLE();

    while (intp_src)
    {
        /*Handle the incomming interrupt. Prioritized.*/
        if ((intp_src & CHB_IRQ_RX_START_MASK))
        {
            intp_src &= ~CHB_IRQ_RX_START_MASK;
        }
        else if (intp_src & CHB_IRQ_TRX_END_MASK)
        {
            state = chb_get_state();

            if ((state == RX_ON) || (state == RX_AACK_ON) || (state == BUSY_RX_AACK))
            {
                // get the ed measurement
                pcb->ed = chb_reg_read(PHY_ED_LEVEL);

                // get the crc
                pcb->crc = (chb_reg_read(PHY_RSSI) & (1<<7)) ? 1 : 0;

                // if the crc is not valid, then do not read the frame and set the rx flag
                if (pcb->crc)
                {
                    // get the data
                    chb_frame_read();
                    pcb->rcvd_xfers++;
                    pcb->data_rcv = true;
                }
            }
            else
            {
                pcb->tx_end = true;
            }
            intp_src &= ~CHB_IRQ_TRX_END_MASK;
            while (chb_set_state(RX_AACK_ON) != RADIO_SUCCESS);
        }
        else if (intp_src & CHB_IRQ_TRX_UR_MASK)
        {
            intp_src &= ~CHB_IRQ_TRX_UR_MASK;
            pcb->underrun++;
        }
        else if (intp_src & CHB_IRQ_PLL_UNLOCK_MASK)
        {
            intp_src &= ~CHB_IRQ_PLL_UNLOCK_MASK;
        }
        else if (intp_src & CHB_IRQ_PLL_LOCK_MASK)
        {
            intp_src &= ~CHB_IRQ_PLL_LOCK_MASK;
        }
        else if (intp_src & CHB_IRQ_BAT_LOW_MASK)
        {
            intp_src &= ~CHB_IRQ_BAT_LOW_MASK;
            pcb->battlow++;
        }
        else
        {
        }
    }
    CHB_LEAVE_CRIT();
}
