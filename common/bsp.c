/*
 * x6100-bbfw -- common BSP implementation (see bsp.h).
 * Verbatim foundation from the Phase 0 bring-up, following Oleg's docs.
 */
#include <stdarg.h>
#include <stdio.h>

#include "bsp.h"

uint32_t SystemCoreClock = 16000000u;      /* HSI reset-time value (clocks.md) */
volatile uint32_t bsp_clk_fail = 0u;

#define CLKF_HSERDY   (1u << 0)
#define CLKF_ODRDY    (1u << 1)
#define CLKF_ODSWRDY  (1u << 2)
#define CLKF_PLLRDY   (1u << 3)
#define CLKF_SWS      (1u << 4)

/* Bounded wait for a register bit to set (never hang on a model/part that
 * fails to raise a ready flag). 1 = set, 0 = timeout. */
static int wait_set(volatile uint32_t *reg, uint32_t mask)
{
    for (uint32_t n = 0; n < 4000000u; n++) {
        if ((*reg) & mask)
            return 1;
    }
    return 0;
}

/* clocks.md -- application clock setup (FUN_08031480). Clock tree chosen by
 * HCLK_MHZ (bsp.h): 250 MHz (PLLCFGR=0x04403C0C + over-drive) by default, or
 * 168 MHz (PLLCFGR=0x07405419, no over-drive). FLASH_ACR=0x705 (5 WS) suits both. */
void bsp_clock_init_250mhz(void)
{
    /* The IAP bootloader hands off with its own PLL already ON (HSI-sourced,
     * ~180 MHz) but SYSCLK still on HSI. PLLCFGR is read-only while PLLON=1,
     * so we must park on HSI and switch the PLL OFF before programming our own
     * config -- otherwise the PLLCFGR write is silently ignored and SYSCLK
     * later lands on the bootloader's frequency, not 250 MHz. (Verified on
     * hardware 2026-10-30 via the clkdiag firmware.) */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY))
        ;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
        ;
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY)
        ;

    RCC->CR |= RCC_CR_HSEON;
    if (!wait_set(&RCC->CR, RCC_CR_HSERDY))
        bsp_clk_fail |= CLKF_HSERDY;

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;

    PWR->CR |= PWR_CR_VOS;                  /* VOS scale 1 */
#if BSP_USE_OVERDRIVE
    PWR->CR |= PWR_CR_ODEN;                 /* >168 MHz: over-drive on top of VOS1 */
    if (!wait_set(&PWR->CSR, PWR_CSR_ODRDY))
        bsp_clk_fail |= CLKF_ODRDY;
    PWR->CR |= PWR_CR_ODSWEN;
    if (!wait_set(&PWR->CSR, PWR_CSR_ODSWRDY))
        bsp_clk_fail |= CLKF_ODSWRDY;
#endif

    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2))
              | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

    RCC->PLLCFGR = BSP_PLLCFGR;

    RCC->CR |= RCC_CR_PLLON;
    if (!wait_set(&RCC->CR, RCC_CR_PLLRDY))
        bsp_clk_fail |= CLKF_PLLRDY;

    FLASH->ACR = 0x00000705u;               /* 5 WS + prefetch + I/D cache */

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    if (!wait_set(&RCC->CFGR, RCC_CFGR_SWS_PLL))
        bsp_clk_fail |= CLKF_SWS;

    SystemCoreClock = HCLK_HZ;
}

/* gpio.md central table + pa_control.md RX-safe/PA-off/charger-off levels.
 * Every documented pin is configured; nothing skipped. */
void bsp_gpio_init_safe(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOB |
                             LL_AHB1_GRP1_PERIPH_GPIOC | LL_AHB1_GRP1_PERIPH_GPIOD |
                             LL_AHB1_GRP1_PERIPH_GPIOE | LL_AHB1_GRP1_PERIPH_GPIOF |
                             LL_AHB1_GRP1_PERIPH_GPIOG);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

    /* Early power/charger (startup.md FUN_08032500): charger OFF. */
    SET_HI(GPIOF, 11);
    SET_LO(GPIOF, 12);                 /* charger enable, active-high: OFF */
    SET_LO(GPIOD, 4);                  /* +5T transmit-rail enable: OFF    */
    M_OUT(GPIOF, 11); M_OUT(GPIOF, 12);
    M_OUT(GPIOD, 4);
    M_IN(GPIOE, 13);
    M_IN(GPIOG, 4);

    /* PA enable tuple -> DISABLED (pa_control.md): PE8=0,PE9=1,PG11=0,PD14=0. */
    SET_LO(GPIOE, 8);
    SET_HI(GPIOE, 9);
    SET_LO(GPIOG, 11);
    SET_LO(GPIOD, 14);
    M_OUT(GPIOE, 8); M_OUT(GPIOE, 9);
    M_OUT(GPIOG, 11);
    M_OUT(GPIOD, 14);

    /* RX-safe routing (pa_control.md "Enter RX / disable TX"). */
    SET_HI(GPIOE, 12);                 /* CW/RF key safe-high        */
    SET_LO(GPIOE, 2);                  /* ACC TRX: RX                */
    SET_HI(GPIOD, 3);                  /* +5R receive rail / green   */
    SET_HI(GPIOD, 2);                  /* support-power / power hold */
    M_OUT(GPIOE, 12); M_OUT(GPIOE, 2);
    M_OUT(GPIOD, 3);  M_OUT(GPIOD, 2);

    /* Remaining central-table outputs. Codec reset PE15 held LOW (asserted). */
    SET_LO(GPIOE, 15);
    SET_LO(GPIOE, 7);
    SET_LO(GPIOE, 10);
    SET_LO(GPIOE, 11);
    SET_HI(GPIOE, 14);                 /* I2C1 device/bus enable (charger.md:185:
                                        * OEM drives PE14 high before I2C1 master
                                        * init); low was the shutdown value and
                                        * left VersaClock 0x6A dark on I2C1.      */
    M_OUT(GPIOE, 7);  M_OUT(GPIOE, 10); M_OUT(GPIOE, 11);
    M_OUT(GPIOE, 14); M_OUT(GPIOE, 15);

    SET_LO(GPIOG, 6);
    SET_LO(GPIOG, 12);
    M_OUT(GPIOG, 6); M_OUT(GPIOG, 12);

    SET_LO(GPIOB, 0); SET_LO(GPIOB, 1); SET_LO(GPIOC, 5);
    M_OUT(GPIOB, 0); SPD_HI(GPIOB, 0);
    M_OUT(GPIOB, 1); SPD_HI(GPIOB, 1);
    M_OUT(GPIOC, 5); SPD_HI(GPIOC, 5);

    SET_LO(GPIOA, 3);
    M_OUT(GPIOA, 3);

    /* Audio speaker-enable (gpio.md): PG1 HPR enable, PG0 loudness; silent in
     * phase 0/1 because the codec (PE15) is held in reset. */
    SET_HI(GPIOG, 0); SET_HI(GPIOG, 1);
    M_OUT(GPIOG, 0); M_OUT(GPIOG, 1);

    /* PG2 = RX front-end enable (the missing analog-path enable). Undocumented in
     * every GPIO table; found by diffing the live OEM GPIO config at 1410 kHz AM
     * (oem-afedump, 2026-11-02) against this bsp: the OEM drives PG2 as an
     * open-drain output with an internal pull-up, released HIGH, in every band
     * (constant RX enable). phase3-rx never configured it, so the whole receive
     * path sat at the noise floor. Bench-proven: driving PG2 high lifts iq_rms on
     * WDOV 1410 kHz AM from ~25 (noise) to ~73-100, with a clean station/empty
     * contrast (env_dc ~90 on WDOV vs ~20 on an empty channel) matching the OEM.
     * Replicate the OEM electrically: open-drain + pull-up, ODR high (released). */
    SET_HI(GPIOG, 2); M_OD(GPIOG, 2); PULL_UP(GPIOG, 2); M_OUT(GPIOG, 2);

    /* Central-table inputs. */
    SET_LO(GPIOG, 8);
    M_IN(GPIOG, 8); PULL_DN(GPIOG, 8);
    M_IN(GPIOA, 11); M_IN(GPIOA, 12);
    M_IN(GPIOB, 8);  M_IN(GPIOB, 9);
    M_IN(GPIOE, 3);
    M_IN(GPIOF, 13); M_IN(GPIOF, 14); M_IN(GPIOF, 15);
    M_IN(GPIOD, 0);

    /* Analog: ADC1 PA0/1/2, ADC2 PA6/7/PC4, DAC PA4/5. */
    M_ANA(GPIOA, 0); M_ANA(GPIOA, 1); M_ANA(GPIOA, 2);
    M_ANA(GPIOA, 4); M_ANA(GPIOA, 5);
    M_ANA(GPIOA, 6); M_ANA(GPIOA, 7);
    M_ANA(GPIOC, 4);

    /* PD12 = TIM4_CH1 (AF2), hardware-key input. */
    LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_12, LL_GPIO_AF_2);
    M_AF(GPIOD, 12);

    /* Explicit safe levels after board init. PD2 high (startup.md FUN_08034178
     * step 5). PF11 HELD HIGH: gpio.md:150 / charger.md:234 ("leave PF11 high on
     * exit") + shutdown.md:39 mark it a power-rail/load-switch enable held high
     * across the running state; the lone "PF11 low" in startup.md step 5 left the
     * codec/analog domain unpowered on hardware. Matches the experiment firmware. */
    SET_HI(GPIOD, 2);
    SET_HI(GPIOF, 11);
}

/* usart.md: USART1 = PA9 TX / PA10 RX, AF7, 115200 8-N-1, from PCLK2=125 MHz. */
void bsp_usart1_init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9,  LL_GPIO_AF_7);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_7);
    M_AF(GPIOA, 9);  SPD_HI(GPIOA, 9);
    M_AF(GPIOA, 10); SPD_HI(GPIOA, 10); PULL_UP(GPIOA, 10);

    LL_USART_Disable(USART1);
    LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
    LL_USART_SetDataWidth(USART1, LL_USART_DATAWIDTH_8B);
    LL_USART_SetStopBitsLength(USART1, LL_USART_STOPBITS_1);
    LL_USART_SetParity(USART1, LL_USART_PARITY_NONE);
    LL_USART_SetHWFlowCtrl(USART1, LL_USART_HWCONTROL_NONE);
    LL_USART_SetOverSampling(USART1, LL_USART_OVERSAMPLING_16);
    LL_USART_SetBaudRate(USART1, PCLK2_HZ, LL_USART_OVERSAMPLING_16, 115200u);
    LL_USART_Enable(USART1);
}

static void usart1_write(const char *s, int len)
{
    for (int i = 0; i < len; i++) {
        while (!LL_USART_IsActiveFlag_TXE(USART1))
            ;
        LL_USART_TransmitData8(USART1, (uint8_t)s[i]);
    }
    while (!LL_USART_IsActiveFlag_TC(USART1))
        ;
}

void bsp_uprintf(const char *fmt, ...)
{
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) {
        if (n > (int)sizeof buf)
            n = (int)sizeof buf;
        usart1_write(buf, n);
    }
}

void bsp_systick_init_1ms(void)
{
    SysTick->LOAD = (SystemCoreClock / 1000u) - 1u;
    SysTick->VAL  = 0u;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

void bsp_delay_ms(uint32_t ms)
{
    while (ms--) {
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
            ;
    }
}
