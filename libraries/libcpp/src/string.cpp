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

#include <string.h>

/** \brief Copy block of memory
 *  \param destination Pointer to the destination array where the content is to be copied.
 *  \param source Pointer to the source of data to be copied.
 *  \param num Number of bytes to copy.
 *  \return returns a pointer to dest.
 */
void *memcpy(void *destination, const void *source, size_t num)
{
  char* d = (char*)destination;
  const char* s = (char*)source;
  while(num)
  {
    *d = *s;
    ++d;
    ++s;
    --num;
  }
  return destination;
}

/** \brief Fill block of memory
 *  \param ptr Pointer to the block of memory to fill.
 *  \param value Value to be set.
 *  \param n Number of bytes to be set to the value.
 *  \return returns ptr.
 */
void *memset(void * ptr, int value, size_t num)
{
  unsigned char* str = (unsigned char*)ptr;

  while (num)
  {
    *str = value;
    ++str;
    --num;
  }
  
  return ptr;
}

