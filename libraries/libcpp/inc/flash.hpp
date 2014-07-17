/*
 * Copyright (c) 2013, Michael E. Ferguson
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

#ifndef _STM32_CPP_FLASH_STORAGE_HPP_
#define _STM32_CPP_FLASH_STORAGE_HPP_

#include <stm32f4xx.h>
#include <stm32f4xx_flash.h>

#define SECTOR0_START_ADDR      0x08000000
#define SECTOR1_START_ADDR      0x08004000
#define SECTOR2_START_ADDR      0x08008000
#define SECTOR3_START_ADDR      0x0800C000
#define SECTOR4_START_ADDR      0x08010000
#define SECTOR5_START_ADDR      0x08020000
#define SECTOR6_START_ADDR      0x08040000
#define SECTOR7_START_ADDR      0x08060000
#define SECTOR8_START_ADDR      0x08080000
#define SECTOR9_START_ADDR      0x080A0000
#define SECTOR10_START_ADDR     0x080C0000
#define SECTOR11_START_ADDR     0x080E0000

/* Size of sectors 0-3 */
#define SECTOR_SIZE_16KB        0x00004000
/* Size of sectos 4 */
#define SECTOR_SIZE_64KB        0x00010000
/* Size of sectors 5-11 */
#define SECTOR_SIZE_128KB       0x00020000

/**
 *  \brief Erases memory page(s).
 *  \param addr The address to start erasing at.
 *  \param len The amount of memory to erase, in 32b words. A length
 *         of -1 indicates to erase to the end of memory.
 *  \returns 0 if OK, -1 if error.
 */
int flash_erase(const uint32_t addr, const int32_t len);

/**
 *  \brief Write data to the flash. NOTE: you need to erase before calling this.
 *  \param addr The address to write data to (should be on the starting address of a page).
 *  \param data Pointer to data to write to flash.
 *  \param data_len Length of data to write, in 32b words.
 *  \returns 0 if OK, -1 if error.
 */
int flash_write(uint32_t addr, uint32_t * data, const int32_t data_len);

/**
 *  \brief Read data from the flash.
 *  \param addr The address to read data from.
 *  \param data Pointer to where to read flash data to.
 *  \param data_len Length of data to be read, in 32b words.
 *  \returns 0 if OK, -1 if error.
 */
int flash_read(uint32_t addr, uint32_t * data, const int32_t data_len);

#endif  // _STM32_CPP_FLASH_STORAGE_HPP_
