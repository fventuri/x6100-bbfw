/*
 * x6100-bbfw -- common startup: vector table, reset handler, SystemInit.
 * startup.md: Initial MSP 0x20030000, vector table 0x08020000; SystemInit
 * enables the FPU (CP10/CP11) and sets VTOR.
 */
#include <stdint.h>
#include "stm32f4xx.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;

extern int main(void);

void Reset_Handler(void);
void Default_Handler(void);

/* SystemInit equivalent (startup.md FUN_0803485c): FPU access + VTOR. */
static void SystemInit_local(void)
{
    SCB->CPACR |= (0xFu << 20);        /* CP10 & CP11 full access (FPU on) */
    __DSB();
    __ISB();
    SCB->VTOR = 0x08020000u;           /* application vector table         */
}

void Reset_Handler(void)
{
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata)
        *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss; )
        *dst++ = 0u;

    SystemInit_local();
    main();

    for (;;)
        ;
}

void Default_Handler(void)
{
    for (;;)
        ;
}

#define WEAK_ALIAS __attribute__((weak, alias("Default_Handler")))
void NMI_Handler(void)        WEAK_ALIAS;
void HardFault_Handler(void)  WEAK_ALIAS;
void MemManage_Handler(void)  WEAK_ALIAS;
void BusFault_Handler(void)   WEAK_ALIAS;
void UsageFault_Handler(void) WEAK_ALIAS;
void SVC_Handler(void)        WEAK_ALIAS;
void DebugMon_Handler(void)   WEAK_ALIAS;
void PendSV_Handler(void)     WEAK_ALIAS;
void SysTick_Handler(void)    WEAK_ALIAS;

/* Peripheral IRQ vectors (STM32F427 position 0..). Phase 2b needs the DMA1
 * Stream3 slot (IRQn 14, vector index 16+14=30) to reach the I2S sample-
 * transport handler. The rest default-alias so earlier phases, which never
 * enable these IRQs, are unaffected. A phase firmware overrides a handler
 * simply by defining a non-weak symbol of the same name. */
void WWDG_IRQHandler(void)          WEAK_ALIAS;  /* IRQ0  */
void PVD_IRQHandler(void)           WEAK_ALIAS;  /* IRQ1  */
void TAMP_STAMP_IRQHandler(void)    WEAK_ALIAS;  /* IRQ2  */
void RTC_WKUP_IRQHandler(void)      WEAK_ALIAS;  /* IRQ3  */
void FLASH_IRQHandler(void)         WEAK_ALIAS;  /* IRQ4  */
void RCC_IRQHandler(void)           WEAK_ALIAS;  /* IRQ5  */
void EXTI0_IRQHandler(void)         WEAK_ALIAS;  /* IRQ6  */
void EXTI1_IRQHandler(void)         WEAK_ALIAS;  /* IRQ7  */
void EXTI2_IRQHandler(void)         WEAK_ALIAS;  /* IRQ8  */
void EXTI3_IRQHandler(void)         WEAK_ALIAS;  /* IRQ9  */
void EXTI4_IRQHandler(void)         WEAK_ALIAS;  /* IRQ10 */
void DMA1_Stream0_IRQHandler(void)  WEAK_ALIAS;  /* IRQ11 */
void DMA1_Stream1_IRQHandler(void)  WEAK_ALIAS;  /* IRQ12 */
void DMA1_Stream2_IRQHandler(void)  WEAK_ALIAS;  /* IRQ13 */
void DMA1_Stream3_IRQHandler(void)  WEAK_ALIAS;  /* IRQ14 */
void I2C3_EV_IRQHandler(void)       WEAK_ALIAS;  /* IRQ72 (Phase 3: Linux control slave) */
void I2C3_ER_IRQHandler(void)       WEAK_ALIAS;  /* IRQ73 */

/* Table sized to the highest used vector (I2C3_EV = idx 16+72 = 88). Positional
 * entries fill idx 0..30 as before; the two designated entries place the I2C3
 * slave vectors. Unlisted slots are 0 -- harmless, since those IRQs are never
 * enabled -- and a phase still overrides any handler by defining its symbol. */
__attribute__((section(".isr_vector"), used))
void (*const g_isr_vector[90])(void) = {
    (void (*)(void))(&_estack),  /* 0x00 Initial MSP */
    Reset_Handler,               /* 0x04 Reset       */
    NMI_Handler,                 /* 0x08 NMI         */
    HardFault_Handler,           /* 0x0C HardFault   */
    MemManage_Handler,           /* 0x10 MemManage   */
    BusFault_Handler,            /* 0x14 BusFault    */
    UsageFault_Handler,          /* 0x18 UsageFault  */
    0, 0, 0, 0,                  /* 0x1C-0x28 reserved */
    SVC_Handler,                 /* 0x2C SVCall      */
    DebugMon_Handler,            /* 0x30 DebugMon    */
    0,                           /* 0x34 reserved    */
    PendSV_Handler,              /* 0x38 PendSV      */
    SysTick_Handler,             /* 0x3C SysTick     */
    /* ---- external (peripheral) interrupts, IRQ0.. ---- */
    WWDG_IRQHandler,             /* idx16 IRQ0  */
    PVD_IRQHandler,              /* idx17 IRQ1  */
    TAMP_STAMP_IRQHandler,       /* idx18 IRQ2  */
    RTC_WKUP_IRQHandler,         /* idx19 IRQ3  */
    FLASH_IRQHandler,            /* idx20 IRQ4  */
    RCC_IRQHandler,              /* idx21 IRQ5  */
    EXTI0_IRQHandler,            /* idx22 IRQ6  */
    EXTI1_IRQHandler,            /* idx23 IRQ7  */
    EXTI2_IRQHandler,            /* idx24 IRQ8  */
    EXTI3_IRQHandler,            /* idx25 IRQ9  */
    EXTI4_IRQHandler,            /* idx26 IRQ10 */
    DMA1_Stream0_IRQHandler,     /* idx27 IRQ11 */
    DMA1_Stream1_IRQHandler,     /* idx28 IRQ12 */
    DMA1_Stream2_IRQHandler,     /* idx29 IRQ13 */
    DMA1_Stream3_IRQHandler,     /* idx30 IRQ14 */
    [88] = I2C3_EV_IRQHandler,   /* idx88 IRQ72 */
    [89] = I2C3_ER_IRQHandler,   /* idx89 IRQ73 */
};
