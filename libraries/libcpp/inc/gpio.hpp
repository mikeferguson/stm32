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

/* 
 * stm32_cpp: a C++ stm32 library
 * This module provides access to a GPIO pin.
 *
 * Usage:
 *  
 *  typedef Gpio<PORTA_BASE, 0> green_led;
 *  ...
 *  green_led::mode(Mode::OUTPUT);
 *  green_led::high();
 *
 */

#ifndef _STM32_CPP_GPIO_H
#define	_STM32_CPP_GPIO_H

#ifdef STM32F4XX

/* IO Modes */
#define GPIO_INPUT                  0x00       /* Floating Input             (MODER=00) */
#define GPIO_INPUT_PULL_UP_DOWN     0x00       /* Pullup/Pulldown Input      (MODER=00) */
#define GPIO_INPUT_ANALOG           0x30       /* Analog Input               (MODER=11) */

#define GPIO_OUTPUT                 0x90       /* Push Pull  50MHz Output    (OTYPER=0 OSPEED=10 MODER=01) */
#define GPIO_OUTPUT_25MHz           0x50       /* Push Pull  25MHz Output    (OTYPER=0 OSPEED=01 MODER=01) */
#define GPIO_OUTPUT_2MHz            0x10       /* Push Pull   2MHz Output    (OTYPER=0 OSPEED=00 MODER=01) */
#define GPIO_OPEN_DRAIN             0x190      /* Open Drain 50MHz Output    (OTYPER=1 OSPEED=10 MODER=01) */
#define GPIO_OPEN_DRAIN_25MHz       0x150      /* Open Drain 25MHz Output    (OTYPER=1 OSPEED=01 MODER=01) */
#define GPIO_OPEN_DRAIN_2MHz        0x110      /* Open Drain  2MHz Output    (OTYPER=1 OSPEED=00 MODER=01) */

#define GPIO_ALTERNATE              0xa0       /* Alternate function 50MHz   (OSPEED=10 OTYPER=0 MODER=10) */
#define GPIO_ALTERNATE_25MHz        0x60       /* Alternate function 25MHz   (OSPEED=01 OTYPER=0 MODER=10) */
#define GPIO_ALTERNATE_2MHz         0x20       /* Alternate function  2MHz   (OSPEED=00 OTYPER=0 MODER=10) */
#define GPIO_ALTERNATE_OD           0x1a0      /* Alternate Open Drain 50MHz (OSPEED=10 OTYPER=1 MODER=10) */
#define GPIO_ALTERNATE_OD_25MHz     0x160      /* Alternate Open Drain 25MHz (OSPEED=01 OTYPER=1 MODER=10) */
#define GPIO_ALTERNATE_OD_2MHz      0x120      /* Alternate Open Drain  2MHz (OSPEED=00 OTYPER=1 MODER=10) */

/* AF0 */
#define GPIO_AF_RTC_50Hz      ((uint8_t)0x00)  /* RTC_50Hz Alternate Function mapping */
#define GPIO_AF_MCO           ((uint8_t)0x00)  /* MCO (MCO1 and MCO2) Alternate Function mapping */
#define GPIO_AF_TAMPER        ((uint8_t)0x00)  /* TAMPER (TAMPER_1 and TAMPER_2) Alternate Function mapping */
#define GPIO_AF_SWJ           ((uint8_t)0x00)  /* SWJ (SWD and JTAG) Alternate Function mapping */
#define GPIO_AF_TRACE         ((uint8_t)0x00)  /* TRACE Alternate Function mapping */

/* AF1 */
#define GPIO_AF_TIM1          ((uint8_t)0x01)  /* TIM1 Alternate Function mapping */
#define GPIO_AF_TIM2          ((uint8_t)0x01)  /* TIM2 Alternate Function mapping */

/* AF2 */
#define GPIO_AF_TIM3          ((uint8_t)0x02)  /* TIM3 Alternate Function mapping */
#define GPIO_AF_TIM4          ((uint8_t)0x02)  /* TIM4 Alternate Function mapping */
#define GPIO_AF_TIM5          ((uint8_t)0x02)  /* TIM5 Alternate Function mapping */

/* AF3 */
#define GPIO_AF_TIM8          ((uint8_t)0x03)  /* TIM8 Alternate Function mapping */
#define GPIO_AF_TIM9          ((uint8_t)0x03)  /* TIM9 Alternate Function mapping */
#define GPIO_AF_TIM10         ((uint8_t)0x03)  /* TIM10 Alternate Function mapping */
#define GPIO_AF_TIM11         ((uint8_t)0x03)  /* TIM11 Alternate Function mapping */

/* AF4 */
#define GPIO_AF_I2C1          ((uint8_t)0x04)  /* I2C1 Alternate Function mapping */
#define GPIO_AF_I2C2          ((uint8_t)0x04)  /* I2C2 Alternate Function mapping */
#define GPIO_AF_I2C3          ((uint8_t)0x04)  /* I2C3 Alternate Function mapping */

/* AF5 */
#define GPIO_AF_SPI1          ((uint8_t)0x05)  /* SPI1 Alternate Function mapping */
#define GPIO_AF_SPI2          ((uint8_t)0x05)  /* SPI2/I2S2 Alternate Function mapping */

/* AF6 */
#define GPIO_AF_SPI3          ((uint8_t)0x06)  /* SPI3/I2S3 Alternate Function mapping */

/* AF7 */
#define GPIO_AF_USART1        ((uint8_t)0x07)  /* USART1 Alternate Function mapping */
#define GPIO_AF_USART2        ((uint8_t)0x07)  /* USART2 Alternate Function mapping */
#define GPIO_AF_USART3        ((uint8_t)0x07)  /* USART3 Alternate Function mapping */
#define GPIO_AF_I2S3ext       ((uint8_t)0x07)  /* I2S3ext Alternate Function mapping */

/* AF8 */
#define GPIO_AF_UART4         ((uint8_t)0x08)  /* UART4 Alternate Function mapping */
#define GPIO_AF_UART5         ((uint8_t)0x08)  /* UART5 Alternate Function mapping */
#define GPIO_AF_USART6        ((uint8_t)0x08)  /* USART6 Alternate Function mapping */

/* AF9 */
#define GPIO_AF_CAN1          ((uint8_t)0x09)  /* CAN1 Alternate Function mapping */
#define GPIO_AF_CAN2          ((uint8_t)0x09)  /* CAN2 Alternate Function mapping */
#define GPIO_AF_TIM12         ((uint8_t)0x09)  /* TIM12 Alternate Function mapping */
#define GPIO_AF_TIM13         ((uint8_t)0x09)  /* TIM13 Alternate Function mapping */
#define GPIO_AF_TIM14         ((uint8_t)0x09)  /* TIM14 Alternate Function mapping */

/* AF10 */
#define GPIO_AF_OTG_FS         ((uint8_t)0xA)  /* OTG_FS Alternate Function mapping */
#define GPIO_AF_OTG_HS         ((uint8_t)0xA)  /* OTG_HS Alternate Function mapping */

/* AF11 */
#define GPIO_AF_ETH            ((uint8_t)0x0B) /* ETHERNET Alternate Function mapping */

/* AF12 */
#define GPIO_AF_FSMC           ((uint8_t)0xC)  /* FSMC Alternate Function mapping */
#define GPIO_AF_OTG_HS_FS      ((uint8_t)0xC)  /* OTG HS configured in FS, Alternate Function mapping */
#define GPIO_AF_SDIO           ((uint8_t)0xC)  /* SDIO Alternate Function mapping */

/* AF13 */
#define GPIO_AF_DCMI           ((uint8_t)0x0D) /* DCMI Alternate Function mapping */

/* AF15*/
#define GPIO_AF_EVENTOUT       ((uint8_t)0x0F) /* EVENTOUT Alternate Function mapping */

#else

/* STM32F1xx IO Modes */
#define GPIO_INPUT                  0x4        /* Floating Input             (CNF=01 MODE=00) */
#define GPIO_INPUT_PULL_UP_DOWN     0x8        /* Pullup/Pulldown Input      (CNF=10 MODE=00) */
#define GPIO_INPUT_ANALOG           0x0        /* Analog Input               (CNF=00 MODE=00) */
#define GPIO_OUTPUT                 0x3        /* Push Pull  50MHz Output    (CNF=00 MODE=11) */
#define GPIO_OUTPUT_10MHz           0x1        /* Push Pull  10MHz Output    (CNF=00 MODE=01) */
#define GPIO_OUTPUT_2MHz            0x2        /* Push Pull   2MHz Output    (CNF=00 MODE=10) */
#define GPIO_OPEN_DRAIN             0x7        /* Open Drain 50MHz Output    (CNF=01 MODE=11) */
#define GPIO_OPEN_DRAIN_10MHz       0x5        /* Open Drain 10MHz Output    (CNF=01 MODE=01) */
#define GPIO_OPEN_DRAIN_2MHz        0x6        /* Open Drain  2MHz Output    (CNF=01 MODE=10) */
#define GPIO_ALTERNATE              0xb        /* Alternate function 50MHz   (CNF=10 MODE=11) */
#define GPIO_ALTERNATE_10MHz        0x9        /* Alternate function 10MHz   (CNF=10 MODE=01) */
#define GPIO_ALTERNATE_2MHz         0xa        /* Alternate function  2MHz   (CNF=10 MODE=10) */
#define GPIO_ALTERNATE_OD           0xf        /* Alternate Open Drain 50MHz (CNF=11 MODE=11) */
#define GPIO_ALTERNATE_OD_10MHz     0xd        /* Alternate Open Drain 10MHz (CNF=11 MODE=01) */
#define GPIO_ALTERNATE_OD_2MHz      0xe        /* Alternate Open Drain  2MHz (CNF=11 MODE=10) */

#endif

#ifdef STM32F4XX
template<unsigned int P, unsigned char N, bool = N >= 8>
struct GpioMode
{
    inline static void mode(unsigned int m)
    {
        reinterpret_cast<GPIO_TypeDef*>(P)->MODER &= ~(0x3<<(N*2));
        reinterpret_cast<GPIO_TypeDef*>(P)->MODER |= ((m>>4)&0x03)<<(N*2);

        reinterpret_cast<GPIO_TypeDef*>(P)->OSPEEDR &= ~(0x3<<(N*2));
        reinterpret_cast<GPIO_TypeDef*>(P)->OSPEEDR |= ((m>>6)&0x03)<<(N*2);

        reinterpret_cast<GPIO_TypeDef*>(P)->OTYPER &= ~(0x1<<N);
        reinterpret_cast<GPIO_TypeDef*>(P)->OTYPER |= ((m>>8)&0x01)<<N;

        reinterpret_cast<GPIO_TypeDef*>(P)->AFR[1] &= ~(0xF<<((N-8)*4));
        reinterpret_cast<GPIO_TypeDef*>(P)->AFR[1] |= (m&0x0F)<<((N-8)*4);
    }
};

template<unsigned int P, unsigned char N>
struct GpioMode<P, N, false>
{
    inline static void mode(unsigned int m)
    {
        reinterpret_cast<GPIO_TypeDef*>(P)->MODER &= ~(0x3<<(N*2));
        reinterpret_cast<GPIO_TypeDef*>(P)->MODER |= ((m>>4)&0x03)<<(N*2);

        reinterpret_cast<GPIO_TypeDef*>(P)->OSPEEDR &= ~(0x3<<(N*2));
        reinterpret_cast<GPIO_TypeDef*>(P)->OSPEEDR |= ((m>>6)&0x03)<<(N*2);

        reinterpret_cast<GPIO_TypeDef*>(P)->OTYPER &= ~(0x1<<N);
        reinterpret_cast<GPIO_TypeDef*>(P)->OTYPER |= ((m>>8)&0x01)<<N;

        reinterpret_cast<GPIO_TypeDef*>(P)->AFR[0] &= ~(0xF<<(N*4));
        reinterpret_cast<GPIO_TypeDef*>(P)->AFR[0] |= (m&0x0F)<<(N*4);
    }
};

#else

template<unsigned int P, unsigned char N, bool = N >= 8>
struct GpioMode
{
    inline static void mode(unsigned int m)
    {
        reinterpret_cast<GPIO_TypeDef*>(P)->CRH &= ~(0xf<<((N-8)*4));
        reinterpret_cast<GPIO_TypeDef*>(P)->CRH |= m<<((N-8)*4);
    }
};

template<unsigned int P, unsigned char N>
struct GpioMode<P, N, false>
{
    inline static void mode(unsigned int m)
    {
        reinterpret_cast<GPIO_TypeDef*>(P)->CRL &= ~(0xf<<(N*4));
        reinterpret_cast<GPIO_TypeDef*>(P)->CRL |= m<<(N*4);
    }
};

#endif

template<unsigned int P, unsigned char N>
class Gpio
{
public:
    static void mode(unsigned int m)
    {
        GpioMode<P, N>::mode(m);
    }

    inline static void high()
    {
#ifdef STM32F4XX
        reinterpret_cast<GPIO_TypeDef*>(P)->BSRRL = 1<<N;
#else
        reinterpret_cast<GPIO_TypeDef*>(P)->BSRR = 1<<N;
#endif
    }

    inline static void low()
    {
#ifdef STM32F4XX
        reinterpret_cast<GPIO_TypeDef*>(P)->BSRRH = 1<<N;
#else
        reinterpret_cast<GPIO_TypeDef*>(P)->BRR = 1<<N;
#endif
    }

    inline static int value()
    {
        return ((reinterpret_cast<GPIO_TypeDef*>(P)->IDR & 1<<N)? 1 : 0);
    }

    inline static void pullup()
    {
#ifdef STM32F4XX
        reinterpret_cast<GPIO_TypeDef*>(P)->PUPDR &= ~(0xf<<(N*2));
        reinterpret_cast<GPIO_TypeDef*>(P)->PUPDR |    0x1<<(N*2);
#else
        high();
#endif
    }

    inline static void pulldown()
    {
#ifdef STM32F4XX
        reinterpret_cast<GPIO_TypeDef*>(P)->PUPDR &= ~(0xf<<(N*2));
        reinterpret_cast<GPIO_TypeDef*>(P)->PUPDR |    0x2<<(N*2);
#else
        low();
#endif
    }

private:
    Gpio(); // disallow
};

#endif	// _STM32_CPP_GPIO_H

