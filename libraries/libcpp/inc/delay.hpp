/*
 * Copyright (c) 2012, Michael E. Ferguson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _STM32_CPP_DELAY_H_
#define _STM32_CPP_DELAY_H_

#ifdef STM32F4XX
  /* 168MHz = each clock cycle is 6ns. Loop is always 6 clock cycles?
   * These can get clock stretched if we have interrupts in the background.
   */
  void delay_ms(const uint16_t ms) 
  {
    uint32_t i = ms * 27778;
	while (i-- > 0) {
		asm("nop");
	}
  }
  void delay_us(const uint16_t us) 
  {
    uint32_t i = us * 28;
	while (i-- > 0) {
		asm("nop");
	}
  }
  void delay_ns(const uint16_t ns) 
  {
    uint32_t i = ns / 36;
	while (i-- > 0) {
		asm("nop");
	}
  }
#else
  /* 72MHz = each clock cycle is 14ns. Loop is always 6 clock cycles?
   * These can get clock stretched if we have interrupts in the background.
   */
  void delay_ms(const uint16_t ms) 
  {
    uint32_t i = ms * 11905;
	while (i-- > 0) {
		asm("nop");
	}
  }
  void delay_us(const uint16_t us) 
  {
    uint32_t i = us * 12;
	while (i-- > 0) {
		asm("nop");
	}
  }
  void delay_ns(const uint16_t ns) 
  {
    uint32_t i = ns / 84;
	while (i-- > 0) {
		asm("nop");
	}
  }
#endif

#endif /* _STM32_CPP_DELAY_H_ */
