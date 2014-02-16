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

#ifndef _STM32_CPP_RCC_H_
#define	_STM32_CPP_RCC_H_

/**
 *  \brief Helper functions for getting clock speeds.
 */
class RccImpl
{
  /* this is a f() conversion of the table used in the std_periph_lib */
  static uint8_t get_prescalar(const uint8_t value){
    if( value < 4 )
      return 0;
    else if( value > 12 )
      return (value & 3) + 6;
    else
      return (value & 3) + 1;    
  }

public:
  static uint32_t get_sysclk(void)
  {
    uint32_t tmp = RCC->CFGR & RCC_CFGR_SWS;
    if (tmp == 0x00)        /* HSI is system clock source */
      return HSI_VALUE;
    else if (tmp == 0x04)   /* HSE used as system clock  source */
      return HSE_VALUE;
    else if (tmp == 0x08)   /* PLL used as system clock  source */
    {
      /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
         SYSCLK = PLL_VCO / PLLP */
      uint32_t pllvco = 0;
      uint32_t pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
      uint32_t pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
      
      if (pllsource != 0)
      {
        /* HSE used as PLL clock source */
        pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }
      else
      {
        /* HSI used as PLL clock source */
        pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);      
      }

      uint32_t pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
      return pllvco/pllp;
    }
    return HSI_VALUE;
  }

  static uint32_t get_hclk(void)
  {
    uint32_t presc = get_prescalar( (RCC->CFGR & RCC_CFGR_HPRE) >> 4 );
    return get_sysclk() >> presc;
  }

  static uint32_t get_pclk1(void)
  {
    uint32_t presc = get_prescalar( (RCC->CFGR & RCC_CFGR_PPRE1) >> 10 );
    return (get_sysclk() >> presc);
  }

  static uint32_t get_pclk2(void)
  {
    uint32_t presc = get_prescalar( (RCC->CFGR & RCC_CFGR_PPRE2) >> 13 );
    return get_sysclk() >> presc;
  }

};

#endif /* _STM32_CPP_RCC_H_ */
