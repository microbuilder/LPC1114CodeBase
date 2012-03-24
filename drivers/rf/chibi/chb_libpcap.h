/**************************************************************************/
/*! 
    @file     chb_libpcap.h
    @author   K. Townsend (microBuilder.eu)
    
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
#ifndef CHB_LIBPCAP_H
#define CHB_LIBPCAP_H

#include "projectconfig.h"

typedef enum libpcap_error
{
  LIBPCAP_OK = 0,
  LIBPCAP_ERROR_NOTINITIALISED = 1,
  LIBPCAP_ERROR_FATFS_NODISK = 10,
  LIBPCAP_ERROR_FATFS_INITFAILED = 11,
  LIBPCAP_ERROR_FATFS_FAILEDTOMOUNTDRIVE = 12,
  LIBPCAP_ERROR_FATFS_UNABLETOCREATEFILE = 13,
  LIBPCAP_ERROR_FATFS_UNABLETOOPENFILE = 14
} libpcap_error_t;

libpcap_error_t libpcapInit(char *filename);
libpcap_error_t libpcapWriteFrame(const uint8_t * frame_buffer, uint32_t frame_len);

#endif
