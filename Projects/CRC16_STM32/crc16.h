/* Copyright (c) 2002, 2003, 2004  Marek Michalkiewicz
   Copyright (c) 2005, 2007 Joerg Wunsch
   Copyright (c) 2013 Dave Hylands
   Copyright (c) 2013 Frederic Nadeau
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

/* $Id$ */

#ifndef _UTIL_CRC16_H_
#define _UTIL_CRC16_H_

#include <stdint.h>

static __inline__ uint16_t
_crc16_update(uint16_t __crc, uint8_t __data)
{
#ifdef STM32
	int i;

	crc ^= a;
	for (i = 0; i < 8; ++i)
	{
	    if (crc & 1)
		crc = (crc >> 1) ^ 0xA001;
	    else
		crc = (crc >> 1);
	}

	return crc;
#else
	uint8_t __tmp;
	uint16_t __ret;

	__asm__ __volatile__ (
		"eor %A0,%2" "\n\t"
		"mov %1,%A0" "\n\t"
		"swap %1" "\n\t"
		"eor %1,%A0" "\n\t"
		"mov __tmp_reg__,%1" "\n\t"
		"lsr %1" "\n\t"
		"lsr %1" "\n\t"
		"eor %1,__tmp_reg__" "\n\t"
		"mov __tmp_reg__,%1" "\n\t"
		"lsr %1" "\n\t"
		"eor %1,__tmp_reg__" "\n\t"
		"andi %1,0x07" "\n\t"
		"mov __tmp_reg__,%A0" "\n\t"
		"mov %A0,%B0" "\n\t"
		"lsr %1" "\n\t"
		"ror __tmp_reg__" "\n\t"
		"ror %1" "\n\t"
		"mov %B0,__tmp_reg__" "\n\t"
		"eor %A0,%1" "\n\t"
		"lsr __tmp_reg__" "\n\t"
		"ror %1" "\n\t"
		"eor %B0,__tmp_reg__" "\n\t"
		"eor %A0,%1"
		: "=r" (__ret), "=d" (__tmp)
		: "r" (__data), "0" (__crc)
		: "r0"
	);
	return __ret;
#endif
}

#endif /* _UTIL_CRC16_H_ */

