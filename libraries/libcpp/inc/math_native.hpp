/*
 * Copyright (c) 2013, Unbounded Robotics Inc.
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

#ifndef _STM32_CPP_MATH_NATIVE_HPP_
#define	_STM32_CPP_MATH_NATIVE_HPP_

#ifdef STM32F4XX
#define M_PI 3.1415926535897931f

inline float native_sqrtf(float input)
{
  float result;
  asm("VSQRT.F32 %[result], %[input]\n"
      : [result] "=w"(result) 
      : [input] "w"(input)
      : );
  return result;
}

inline float native_fabsf(float input)
{
  float result;
  asm("VABS.F32 %[result],%[input]\n"
      : [result] "=w"(result)
      : [input] "w"(input)
      : "cc");
  return result;
}

/** \brief Returns true is floating point value is not +/-infinity or NaN */
inline bool native_isfinite(float input)
{
  /* Both NaN and infinity is denoted by exponent value of all ones
   * In IEEE 754 single precision format, exponent is bits 23:30 of floating point value
   * http://en.wikipedia.org/wiki/Single_precision
   */

  /* We need to access the bits 23:30 of the floating point value directly
   * however playing with pointer magic like:
   *    uint32t value =*reinterpret_cast<uint32_t*>(&input);
   * produces the following compile time warning:
   *    dereferencing type-punned pointer will break strict-aliasing rules
   * 
   * AFAIK this means the compile could optimize the code in odd ways.
   * This is definately not what we want, especially since this function is used 
   * for error checking so it might not be obvious the code was optimized out
   * 
   * The other issue with pointer dereferencing, it is assumes and/or forces the 
   * floating point value to be stored in memory.  
   * The floating point value is mostly likely in a floating point register.
   * 
   * My solution is to use a mov instruction to move the value in a floating
   * point register to a normal 32bit register.  
   * This prevents the compiler from trying to optimize anything. *
   * The value in the normal register will have the same binary value as the 
   * value in the floating point register.
   * This is not the same as C code equivalent of
   *    uint32_t value = static_cast<uint32_t>(input)
   * This will actually used the VCVT instruction which will round the floating 
   * point value to an integer value (changing the binary value).
   */
  uint32_t value;
  asm("VMOV %[value],%[input]\n"
      : [value] "=r"(value)
      : [input] "w"(input)
      : "cc");
  return (value & 0x7F800000) != 0x7F800000;
}

/**
 *  \brief Converts binary value to single precision float directly 
 * 
 *  Useful for generating NaNs and Inf.
 *    ie : convert_binary_to_float(0xFF800000) is negative infinity
 */
inline float gen_float(uint32_t input)
{
  float result;
  asm("VMOV %[result],%[input]\n"
      : [result] "=w"(result)
      : [input] "r"(input)
      : "cc");
  return result;
}

/** \brief Converts single precision float to binary representation of float. */
inline uint32_t convert_float_to_binary(float input)
{
  uint32_t result;
  asm("VMOV %[result],%[input]\n"
      : [result] "=r"(result)
      : [input] "w"(input)
      : "cc");
  return result;
}
#endif

#endif  // _STM32_CPP_MATH_NATIVE_HPP_
