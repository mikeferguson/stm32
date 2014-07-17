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

#include <flash.hpp>

#define BYTES_PER_WORD	4

int get_sector_num(const uint32_t addr)
{
  if (addr >= SECTOR5_START_ADDR)
  {
    int sector = 5 + ((addr - SECTOR5_START_ADDR)/SECTOR_SIZE_128KB);
    if (sector > 11)
      return 11;
    return sector;
  }
  else if (addr >= SECTOR4_START_ADDR)
    return 4;
  else
    return (addr - SECTOR0_START_ADDR)/SECTOR_SIZE_16KB;
}

int flash_erase(const uint32_t addr, const int32_t len)
{
  int start_sector, end_sector;

  /* Check bounds */
  if (addr < SECTOR0_START_ADDR)
    return -1;

  /* Need to compute sector to erase */
  start_sector = get_sector_num(addr);
  if (len == -1)
    end_sector = 11;
  else
    end_sector = get_sector_num(addr+(len*BYTES_PER_WORD)-1);

  FLASH_Unlock();
  for (int i = start_sector; i <= end_sector; ++i)
  {
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
    FLASH_EraseSector(i * 0x08, VoltageRange_3);
  }
  FLASH_Lock();
  return 0;
}

int flash_write(uint32_t addr, uint32_t * data, const int32_t data_len)
{
  FLASH_Unlock();

  /* Write data */
  uint32_t * d = data;
  for (int i = 0; i < data_len; ++i)
  {
    if (FLASH_ProgramWord(addr+(i*BYTES_PER_WORD), *d) != FLASH_COMPLETE)
      return -1;
    /* Confirm success */
    if (*(uint32_t*)(addr+(i*BYTES_PER_WORD)) != *d)
      return -1;
    ++d;
  }

  FLASH_Lock();
  return 0;
}

int flash_read(uint32_t addr, uint32_t * data, const int32_t data_len)
{
  uint32_t * d = data;
  for (int i = 0; i < data_len; ++i)
  {
    *d = *(volatile uint32_t*)(addr+(i*BYTES_PER_WORD));
    ++d;
  }
  return 0;
}
