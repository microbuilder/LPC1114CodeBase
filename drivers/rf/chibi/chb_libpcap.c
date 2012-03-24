/**************************************************************************/
/*! 
    @file     chb_libpcap.c
    @author   Based on wsbridge by Christopher Wang (Freaklabs)
              Modified by: K. Townsend (microBuilder.eu)
    
    @brief    Writes raw 802.15.4 frames to a binary file using the
              widely-adopted libpcap format (used by Wireshark and other
              protocal analysers).

    @note     Chibi must be in PROMISCUOUS mode for this to function,
              otherwise an incorrect frame length will be reported
              (the size of the message payload, not the entire frame).
              To enable PROMISCUOUS mode, set CFG_CHIBI_PROMISCUOUS to
              '1' in 'projectconfig.h'.

    @section Example

    @code
    #include "drivers/chibi/chb.h"
    #include "drivers/chibi/chb_drvr.h"
    #include "drivers/chibi/chb_libpcap.h"
    static chb_rx_data_t rx_data;
    ...
    int main (void)
    {
      // Configure cpu and mandatory peripherals
      systemInit();

      #if defined CFG_CHIBI && defined CFG_SDCARD && CFG_CHIBI_PROMISCUOUS != 0    
        // Get a reference to the Chibi peripheral control block
        chb_pcb_t *pcb = chb_get_pcb();

        // Create a binary file to store captured data
        libpcapInit("/capture.cap");
        
        // Wait for incoming frames and log them to disk in libpcap format.
        while(1)
        {
          // Check for incoming messages 
          while (pcb->data_rcv) 
          { 
            // get the length of the data
            rx_data.len = chb_read(&rx_data);
            // make sure the length is non-zero
            if (rx_data.len)
            {
              // Enable LED to indicate message reception 
              gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_ON); 
              // Write frame content to disk
              libpcapWriteFrame(rx_data.data, rx_data.len);
              // Disable LED
              gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_OFF); 
            }
          }
        }
      #endif

      // Create a binary file to store captured data
      libpcapInit("capture.cap");
  
      // Wait for incoming frames and log them to disk in libpcap format.
      chb_pcb_t *pcb = chb_get_pcb();
      while(1)
      {
        // Check for incoming messages 
        while (pcb->data_rcv) 
        { 
          // get the length of the data
          rx_data.len = chb_read(&rx_data);
          // make sure the length is non-zero
          if (rx_data.len)
          {
            // Enable LED to indicate message reception 
            gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_ON); 
            // Write frame content to disk
            libpcapWriteFrame(rx_data.data, rx_data.len);
            // Disable LED
            gpioSetValue (CFG_LED_PORT, CFG_LED_PIN, CFG_LED_OFF); 
          }
        }
      }
    }

    @endcode

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2011, microBuilder SARL
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
#include <string.h>
#include "chb_libpcap.h"
#include "core/systick/systick.h"

#define LIBPCAP_LOCALFILE (0)
#define LIBPCAP_FATFSFILE (1)

// Write local files using crossworks debug library (CW Debug only)
#if LIBPCAP_LOCALFILE == 1
  #include <cross_studio_io.h>
  DEBUG_FILE * libpcapLocalFile;
#endif

// Write files to SD using FatFS
#if defined CFG_SDCARD && LIBPCAP_FATFSFILE == 1
  #include "core/ssp/ssp.h"
  #include "drivers/fatfs/diskio.h"
  #include "drivers/fatfs/ff.h"
  static FILINFO Finfo;
  static FATFS Fatfs[1];
  static uint8_t buf[64]; 
  static FIL libpcapSDFile;
#endif

#define LIBPCAP_FCSLENGTH         2

char * libpcapFName;
static uint32_t libpcapStartTick;
static bool libpcapInitialised = FALSE;

/**************************************************************************/
/*! 
    @brief Writes the top-level libpcap header to the file
*/
/**************************************************************************/
void libpcapWriteGlobalHeader()
{
  uint32_t magic_num = 0xa1b2c3d4;  // used for endianness
  uint16_t version_major = 2;       // version
  uint16_t version_minor = 4;       // version
  int32_t  thiszone = 0;            // zone (unused)
  uint32_t sigfigs = 0;             // significant figures (unused)
  uint32_t snaplen = 65535;         // snapshot length (max value)
  uint32_t network = 195;           // Data Link Type (DLT): indicates link layer protocol

  // Write global header to disk
  #if LIBPCAP_LOCALFILE == 1
    debug_fwrite(&magic_num, 4, 1, libpcapLocalFile);
    debug_fwrite(&version_major, 2, 1, libpcapLocalFile);
    debug_fwrite(&version_minor, 2, 1, libpcapLocalFile);
    debug_fwrite(&thiszone, 4, 1, libpcapLocalFile);
    debug_fwrite(&sigfigs, 4, 1, libpcapLocalFile);
    debug_fwrite(&snaplen, 4, 1, libpcapLocalFile);
    debug_fwrite(&network, 4, 1, libpcapLocalFile);
  #endif
  #if defined CFG_SDCARD && LIBPCAP_FATFSFILE == 1
    unsigned int bytesWritten;
    f_write(&libpcapSDFile, &magic_num, 4, &bytesWritten);
    f_write(&libpcapSDFile, &version_major, 2, &bytesWritten);
    f_write(&libpcapSDFile, &version_minor, 2, &bytesWritten);
    f_write(&libpcapSDFile, &thiszone, 4, &bytesWritten);
    f_write(&libpcapSDFile, &sigfigs, 4, &bytesWritten);
    f_write(&libpcapSDFile, &snaplen, 4, &bytesWritten);
    f_write(&libpcapSDFile, &network, 4, &bytesWritten);
    // Flush the write buffer
    f_sync(&libpcapSDFile);
  #endif
}

/**************************************************************************/
/*! 
    @brief Writes the libpcap frame header that preceeds each frame
*/
/**************************************************************************/
void libpcapWriteFrameHeader(uint32_t sec, uint32_t usec, uint32_t incl_len, uint32_t orig_len)
{
  #if LIBPCAP_LOCALFILE == 1
    debug_fwrite(&sec, 4, 1, libpcapLocalFile);
    debug_fwrite(&usec, 4, 1, libpcapLocalFile);
    debug_fwrite(&incl_len, 4, 1, libpcapLocalFile);
    debug_fwrite(&orig_len, 4, 1, libpcapLocalFile);
  #endif
  #if defined CFG_SDCARD && LIBPCAP_FATFSFILE == 1
    unsigned int bytesWritten;
    f_write(&libpcapSDFile, &sec, 4, &bytesWritten);
    f_write(&libpcapSDFile, &usec, 4, &bytesWritten);
    f_write(&libpcapSDFile, &incl_len, 4, &bytesWritten);
    f_write(&libpcapSDFile, &orig_len, 4, &bytesWritten);
  #endif
}

/**************************************************************************/
/*! 
    @brief Writes an entire libpcap frame, including the frame header
*/
/**************************************************************************/
libpcap_error_t libpcapWriteFrame(const uint8_t * frame_buffer, uint32_t frame_len)
{
    if (!libpcapInitialised)
    {
      return LIBPCAP_ERROR_NOTINITIALISED;
    }

    uint32_t incl_len, orig_len;
    long sec, usec;

    // Generate a timestamp
    // Since we have no RTC, count ticks from when program was started
    uint32_t currentTick = systickGetTicks();
    long diff_in_ticks = currentTick - libpcapStartTick;        // get difference in ticks
    sec = diff_in_ticks / (1000 / CFG_SYSTICK_DELAY_IN_MS);     // get seconds
    diff_in_ticks -= (sec * (1000 / CFG_SYSTICK_DELAY_IN_MS));  // subtract off seconds from total
    usec = currentTick * 1000;                                  // get usec

    // ToDo: Update the code above to take into consideration 
    // integer overflow!

    // Calculate frame length (minus frame checksum [FCS])
    incl_len = (uint32_t)frame_len - LIBPCAP_FCSLENGTH;
    orig_len = frame_len;

    // Open file for binary append
    #if LIBPCAP_LOCALFILE == 1
      libpcapLocalFile = debug_fopen(libpcapFName, "ab");
    #endif
    #if defined CFG_SDCARD && LIBPCAP_FATFSFILE == 1
      if(f_open(&libpcapSDFile, libpcapFName, FA_READ | FA_WRITE | FA_OPEN_EXISTING)!=FR_OK) 
      {
        return LIBPCAP_ERROR_FATFS_UNABLETOOPENFILE;
      }
      // Move to end of the file to append data
      f_lseek(&libpcapSDFile, (&libpcapSDFile)->fsize);
    #endif

    // write frame header
    libpcapWriteFrameHeader(sec, usec, incl_len, orig_len);

    // Loop through frame length and write data (skipping byte 0, frame length)
    uint8_t byte;
    for (int i = 1; i < incl_len + 1; i++)
    {
        byte = frame_buffer[i];
        #if LIBPCAP_LOCALFILE == 1
          debug_fwrite(&byte, 1, 1, libpcapLocalFile);
        #endif
        #if defined CFG_SDCARD && LIBPCAP_FATFSFILE == 1
          unsigned int bytesWritten;
          f_write(&libpcapSDFile, &byte, 1, &bytesWritten);
        #endif
    }

    #if LIBPCAP_LOCALFILE == 1
      debug_fclose(libpcapLocalFile);
    #endif
    #if defined CFG_SDCARD && LIBPCAP_FATFSFILE == 1
      f_sync(&libpcapSDFile);
      f_close(&libpcapSDFile);
    #endif

    return LIBPCAP_OK;
}

/**************************************************************************/
/*! 
    @brief Initialises a new libpcap binary file and writes the header
*/
/**************************************************************************/
libpcap_error_t libpcapInit(char *filename)
{
  libpcapFName = filename;
  libpcapStartTick = systickGetTicks();

  // Create a new file
  #if LIBPCAP_LOCALFILE == 1
    libpcapLocalFile = debug_fopen(libpcapFName, "wb");
  #endif
  #if defined CFG_SDCARD && LIBPCAP_FATFSFILE == 1
    DSTATUS stat;
    stat = disk_initialize(0);
    if (stat & STA_NOINIT) 
    {
      return LIBPCAP_ERROR_FATFS_INITFAILED;
    }
    if (stat & STA_NODISK) 
    {
      return LIBPCAP_ERROR_FATFS_NODISK;
    }
    if (stat == 0)
    {
      // SD card sucessfully initialised
      DSTATUS stat;
      DWORD p2;
      WORD w1;
      BYTE res, b1;
      DIR dir; 
      // Try to mount drive
      res = f_mount(0, &Fatfs[0]);
      if (res != FR_OK) 
      {
        return LIBPCAP_ERROR_FATFS_FAILEDTOMOUNTDRIVE;
      }
      if (res == FR_OK)
      {
        // Create a file (overwriting any existing file!)
        if(f_open(&libpcapSDFile, libpcapFName, FA_READ | FA_WRITE | FA_CREATE_ALWAYS)!=FR_OK) 
        {  
          return LIBPCAP_ERROR_FATFS_UNABLETOCREATEFILE; 
        }
      }
    }
  #endif

  // Write libpcap header
  libpcapWriteGlobalHeader();

  // Close the file (not a great idea to keep it open permanently)
  #if LIBPCAP_LOCALFILE == 1
    debug_fclose(libpcapLocalFile);
  #endif
  #if defined CFG_SDCARD && LIBPCAP_FATFSFILE == 1
    f_close(&libpcapSDFile);
    // ToDo: This will leave the driver mounted ... when to call "f_mount(0,0)"?
  #endif

  libpcapInitialised = TRUE;
  return LIBPCAP_OK;
}
