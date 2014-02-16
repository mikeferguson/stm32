/*
 * Copyright (c) 2012-2014, Michael E. Ferguson
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

#include "stm32f4xx.h"

/**
 *  \defgroup DelayH Delay Routines
 *  These routines can be found by including delay.hpp. Additionally, you
 *  will have to build delay.cpp in your Makefile.
 *
 *  Note that these routines are only designed for 168Mhz clock rates, and
 *  that delay routines may take longer than desired if the processor is
 *  under heavy load from interrupts.
 *  \{
 */

/**
 *  \brief Delay for a number of milliseconds.
 *  \param ms The number of milliseconds to delay.
 */
void delay_ms(const uint16_t ms);

/**
 *  \brief Delay for a number of microseconds.
 *  \param us The number of microseconds to delay.
 */
void delay_us(const uint16_t us);

/**
 *  \brief Delay for a number of nanoseconds.
 *  \param ns The number of nanoseconds to delay.
 */
void delay_ns(const uint16_t ns);

/**
 *  \}
 */

#endif /* _STM32_CPP_DELAY_H_ */
