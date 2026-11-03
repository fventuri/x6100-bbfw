/*
 * x6100-bbfw -- common board-support package (BSP)
 * ================================================
 * Shared foundation extracted from Phase 0, used by every phase firmware:
 * reset/vector table, FPU+VTOR init, the OEM 250 MHz clock tree, the full
 * GPIO safe-state, USART1 diagnostics, and a polled 1 ms SysTick timebase.
 *
 * Peripheral access uses the STM32 Low-Level (LL) inline API; the clock tree
 * is programmed with direct CMSIS register writes to match Oleg's VERIFIED
 * constants (x6100-base/docs/clocks.md).
 */
#ifndef X6100_BSP_H
#define X6100_BSP_H

#include <stdint.h>

#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"

/* Core clock selected at build time (clocks.md). Default 250 MHz = Oleg's OEM
 * tree; build with -DHCLK_MHZ=168 (e.g. `HCLK_MHZ=168 ./build.sh`) for the
 * in-spec safe clock. Both keep PCLK1=HCLK/4, PCLK2=HCLK/2. */
#ifndef HCLK_MHZ
#define HCLK_MHZ 250
#endif

#if   HCLK_MHZ == 250
#  define HCLK_HZ           250000000u
#  define PCLK1_HZ           62500000u   /* HCLK/4 */
#  define PCLK2_HZ          125000000u   /* HCLK/2 */
#  define BSP_PLLCFGR       0x04403C0Cu  /* HSE 25M, M12 N240 P2 Q4 -> 250 MHz */
#  define BSP_USE_OVERDRIVE 1            /* >168 MHz requires over-drive       */
#  define BSP_CLK_DESC      "OEM 250 MHz tree"
#elif HCLK_MHZ == 168
#  define HCLK_HZ           168000000u
#  define PCLK1_HZ           42000000u   /* HCLK/4 */
#  define PCLK2_HZ           84000000u   /* HCLK/2 */
#  define BSP_PLLCFGR       0x07405419u  /* HSE 25M, M25 N336 P2 Q7 -> 168 MHz */
#  define BSP_USE_OVERDRIVE 0            /* <=168 MHz: VOS scale 1, no OD       */
#  define BSP_CLK_DESC      "168 MHz safe clock, in-spec"
#else
#  error "unsupported HCLK_MHZ (use 250 or 168)"
#endif

extern uint32_t SystemCoreClock;

/* Which PLL/over-drive "ready" waits timed out during clock init (0 = OK).
 * bit0 HSERDY, 1 ODRDY, 2 ODSWRDY, 3 PLLRDY, 4 SWS. */
extern volatile uint32_t bsp_clk_fail;

void bsp_clock_init_250mhz(void);  /* HSE PLL per HCLK_MHZ (250 w/ OD, or 168); clocks.md */
void bsp_gpio_init_safe(void);     /* full central GPIO table + RX-safe levels   */
void bsp_usart1_init(void);        /* PA9/PA10 AF7, 115200 8-N-1 (usart.md)       */
void bsp_uprintf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void bsp_systick_init_1ms(void);
void bsp_delay_ms(uint32_t ms);

/* Compact LL GPIO helpers keyed by pin bit number (0..15); LL_GPIO_PIN_n == 1<<n. */
#define M_OUT(port, bit) LL_GPIO_SetPinMode((port), 1u << (bit), LL_GPIO_MODE_OUTPUT)
#define M_IN(port, bit)  LL_GPIO_SetPinMode((port), 1u << (bit), LL_GPIO_MODE_INPUT)
#define M_ANA(port, bit) LL_GPIO_SetPinMode((port), 1u << (bit), LL_GPIO_MODE_ANALOG)
#define M_AF(port, bit)  LL_GPIO_SetPinMode((port), 1u << (bit), LL_GPIO_MODE_ALTERNATE)
#define M_OD(port, bit)  LL_GPIO_SetPinOutputType((port), 1u << (bit), LL_GPIO_OUTPUT_OPENDRAIN)
#define M_PP(port, bit)  LL_GPIO_SetPinOutputType((port), 1u << (bit), LL_GPIO_OUTPUT_PUSHPULL)
#define SET_HI(port, bit) LL_GPIO_SetOutputPin((port), 1u << (bit))
#define SET_LO(port, bit) LL_GPIO_ResetOutputPin((port), 1u << (bit))
#define PULL_UP(port, bit) LL_GPIO_SetPinPull((port), 1u << (bit), LL_GPIO_PULL_UP)
#define PULL_DN(port, bit) LL_GPIO_SetPinPull((port), 1u << (bit), LL_GPIO_PULL_DOWN)
#define SPD_HI(port, bit)  LL_GPIO_SetPinSpeed((port), 1u << (bit), LL_GPIO_SPEED_FREQ_HIGH)

#endif /* X6100_BSP_H */
