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
 * STM32F4XX Startup Code - Entirely in C!
 * author: Michael E. Ferguson
 */

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif 

extern unsigned int __estack;
extern unsigned int __etext;
extern unsigned int __sdata;
extern unsigned int __edata;
extern unsigned int __sbss;
extern unsigned int __ebss;
 
extern int main(void);
extern void SystemInit(void);
 
/* Cortex M4 core interrupt handlers */
void Reset_Handler(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DebugMon_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void PendSV_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
void SysTick_Handler(void) __attribute__ ((weak, alias ("Default_Handler")));
/* STM32F4 specific interrupt handlers */
void WWDG_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void PVD_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void TAMPER_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void RTC_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void FLASH_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void RCC_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void EXTI0_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void EXTI1_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void EXTI2_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void EXTI3_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void EXTI4_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Stream0_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Stream1_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Stream2_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Stream3_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Stream4_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Stream5_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Stream6_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void ADC_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void USB_HP_CAN1_TX_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void USB_LP_CAN1_RX0_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void CAN1_RX1_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void CAN1_SCE_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void EXTI9_5_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void TIM1_BRK_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void TIM1_UP_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void TIM1_TRG_COM_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void TIM1_CC_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void TIM2_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void TIM3_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void TIM4_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void I2C1_EV_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void I2C1_ER_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void I2C2_EV_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void I2C2_ER_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void SPI1_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void SPI2_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void USART1_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void USART2_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void USART3_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void EXTI15_10_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void RTCAlarm_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void USBWakeUp_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void TIM8_BRK_TIM12_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void TIM8_UP_TIM13_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void TIM8_TRG_COM_TIM14_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void TIM8_CC_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Stream7_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void FSMC_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void SDIO_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void TIM5_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void SPI3_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void UART4_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void UART5_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void TIM6_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void TIM7_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_Stream0_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_Stream1_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_Stream2_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_Stream3_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_Stream4_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void ETH_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void ETH_WKUP_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void CAN2_TX_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void CAN2_RX0_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void CAN2_RX1_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void CAN2_SCE_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void OTG_FS_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_Stream5_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_Stream6_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_Stream7_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void USART6_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void I2C3_EV_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void I2C3_ER_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void OTG_HS_EP1_OUT_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void OTG_HS_EP1_IN_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void OTG_HS_WKUP_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void OTG_HS_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void DCMI_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void CRYP_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void HASH_RNG_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));
void FPU_IRQHandler(void) __attribute__ ((weak, alias ("Default_Handler")));

void Default_Handler(void);
 
void *vector_table[] __attribute__ ((section("vectors"))) = {
  /* CM4 core interrupts */
    &__estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,
    0,
    0,
    0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
  /* Peripheral interrupts */
    WWDG_IRQHandler,
    PVD_IRQHandler,
    TAMPER_IRQHandler,
    RTC_IRQHandler,
    FLASH_IRQHandler,
    RCC_IRQHandler,
    EXTI0_IRQHandler,
    EXTI1_IRQHandler,
    EXTI2_IRQHandler,
    EXTI3_IRQHandler,
    EXTI4_IRQHandler,
    DMA1_Stream0_IRQHandler,
    DMA1_Stream1_IRQHandler,
    DMA1_Stream2_IRQHandler,
    DMA1_Stream3_IRQHandler,
    DMA1_Stream4_IRQHandler,
    DMA1_Stream5_IRQHandler,
    DMA1_Stream6_IRQHandler,
    ADC_IRQHandler,
    USB_HP_CAN1_TX_IRQHandler,
    USB_LP_CAN1_RX0_IRQHandler,
    CAN1_RX1_IRQHandler,
    CAN1_SCE_IRQHandler,
    EXTI9_5_IRQHandler,
    TIM1_BRK_IRQHandler,
    TIM1_UP_IRQHandler,
    TIM1_TRG_COM_IRQHandler,
    TIM1_CC_IRQHandler,
    TIM2_IRQHandler,
    TIM3_IRQHandler,
    TIM4_IRQHandler,
    I2C1_EV_IRQHandler,
    I2C1_ER_IRQHandler,
    I2C2_EV_IRQHandler,
    I2C2_ER_IRQHandler,
    SPI1_IRQHandler,
    SPI2_IRQHandler,
    USART1_IRQHandler,
    USART2_IRQHandler,
    USART3_IRQHandler,
    EXTI15_10_IRQHandler,
    RTCAlarm_IRQHandler,
    USBWakeUp_IRQHandler,
    TIM8_BRK_TIM12_IRQHandler,
    TIM8_UP_TIM13_IRQHandler,
    TIM8_TRG_COM_TIM14_IRQHandler,
    TIM8_CC_IRQHandler,
    DMA1_Stream7_IRQHandler,
    FSMC_IRQHandler,
    SDIO_IRQHandler,
    TIM5_IRQHandler,
    SPI3_IRQHandler,
    UART4_IRQHandler,
    UART5_IRQHandler,
    TIM6_IRQHandler,
    TIM7_IRQHandler,
    DMA2_Stream0_IRQHandler,
    DMA2_Stream1_IRQHandler,
    DMA2_Stream2_IRQHandler,
    DMA2_Stream3_IRQHandler,
    DMA2_Stream4_IRQHandler,
    ETH_IRQHandler,
    ETH_WKUP_IRQHandler,
    CAN2_TX_IRQHandler,
    CAN2_RX0_IRQHandler,
    CAN2_RX1_IRQHandler,
    CAN2_SCE_IRQHandler,
    OTG_FS_IRQHandler,
    DMA2_Stream5_IRQHandler,
    DMA2_Stream6_IRQHandler,
    DMA2_Stream7_IRQHandler,
    USART6_IRQHandler,
    I2C3_EV_IRQHandler,
    I2C3_ER_IRQHandler,
    OTG_HS_EP1_OUT_IRQHandler,
    OTG_HS_EP1_IN_IRQHandler,
    OTG_HS_WKUP_IRQHandler,
    OTG_HS_IRQHandler,
    DCMI_IRQHandler,
    CRYP_IRQHandler,
    HASH_RNG_IRQHandler,
    FPU_IRQHandler
};

/* init/fini arrays */
extern void (*__preinit_array_start []) (void) __attribute__((weak));
extern void (*__preinit_array_end []) (void) __attribute__((weak));
extern void (*__init_array_start []) (void) __attribute__((weak));
extern void (*__init_array_end []) (void) __attribute__((weak));
extern void (*__fini_array_start []) (void) __attribute__((weak));
extern void (*__fini_array_end []) (void) __attribute__((weak));

void Reset_Handler(void) {
    unsigned int *src, *dst;
     
    /* Copy data section from flash to RAM */
    src = &__etext;
    dst = &__sdata;
    while (dst < &__edata)
        *dst++ = *src++;

    /* Clear the bss section */
    dst = &__sbss;
    while (dst < &__ebss)
        *dst++ = 0;

    /* Setup clocks and whatnot */
    SystemInit();

    /* This is basically libc_init_array -- handles global constructors */
    unsigned int count;
    unsigned int i;

    count = __preinit_array_end - __preinit_array_start;
    for (i = 0; i < count; i++)
      __preinit_array_start[i] ();

    count = __init_array_end - __init_array_start;
    for (i = 0; i < count; i++)
      __init_array_start[i] ();

    main();
}  

void Default_Handler(void) {
    while (1)
        ;
}

void NMI_Handler(void){}

void HardFault_Handler(void) {
    while (1)
        ;
}

void MemManage_Handler(void) {
    while (1)
        ;
}

void BusFault_Handler(void) {
    while (1)
        ;
}

void UsageFault_Handler(void) {
    while (1)
        ;
}

#ifdef __cplusplus
 }
#endif 

