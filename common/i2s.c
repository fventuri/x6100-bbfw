/*
 * x6100-bbfw -- Phase 2b: I2S sample transport implementation.
 * See i2s.h for the architecture summary; x6100-base/docs/spi_i2s.md + dma.md
 * for the recovered contract (FUN_080320fc config, FUN_08024528 handler).
 */
#include "i2s.h"
#include "bsp.h"

/* ---- DSP rate constants (FUN_08021744), published for Phase 3 ---- */
volatile float i2s_rate_raw     = 100000.0f;
volatile float i2s_rate_primary =  12500.0f;
volatile float i2s_rate_audio   =  12500.0f;
volatile float i2s_rate_alt     =  25000.0f;

/* ---- DMA double-buffer banks (our own RAM; OEM used 0x2000C6F4..0x2000E2F4).
 * 4-byte aligned; each bank is 512 halfwords = 0x400 bytes. ---- */
static uint16_t rx_iq_m0[I2S_BANK_HALFWORDS]    __attribute__((aligned(4))); /* S0 SPI3 RX */
static uint16_t rx_iq_m1[I2S_BANK_HALFWORDS]    __attribute__((aligned(4)));
static uint16_t rx_audio_m0[I2S_BANK_HALFWORDS] __attribute__((aligned(4))); /* S3 SPI2 RX */
static uint16_t rx_audio_m1[I2S_BANK_HALFWORDS] __attribute__((aligned(4)));
static uint16_t tx_audio_m0[I2S_BANK_HALFWORDS] __attribute__((aligned(4))); /* S4 I2S2ext */
static uint16_t tx_audio_m1[I2S_BANK_HALFWORDS] __attribute__((aligned(4)));
static uint16_t tx_iq_m0[I2S_BANK_HALFWORDS]    __attribute__((aligned(4))); /* S5 I2S3ext */
static uint16_t tx_iq_m1[I2S_BANK_HALFWORDS]    __attribute__((aligned(4)));

volatile uint16_t *i2s_rx_audio;
volatile uint16_t *i2s_rx_iq;
volatile uint16_t *i2s_tx_audio;
volatile uint16_t *i2s_tx_iq;
volatile uint32_t  i2s_bank_count;
volatile uint32_t  i2s_isr_entries;
volatile int32_t   i2s_last_audio[2];
volatile int32_t   i2s_last_iq[2];

/* I2SCFGR value shared by both receive blocks: slave receive (I2SCFG=01),
 * Philips (I2SSTD=00), low polarity (CKPOL=0), 32-bit data (DATLEN=10) in a
 * 32-bit channel (CHLEN=1), I2S mode. I2SE left clear. */
#define I2SCFGR_RX ( SPI_I2SCFGR_I2SMOD                     \
                   | SPI_I2SCFGR_I2SCFG_0 /* 01 slave RX */ \
                   | SPI_I2SCFGR_DATLEN_1 /* 10 = 32-bit */ \
                   | SPI_I2SCFGR_CHLEN )
/* Ext blocks: complementary slave transmit (I2SCFG=00); same word format. */
#define I2SCFGR_TX ( SPI_I2SCFGR_I2SMOD                     \
                   | SPI_I2SCFGR_DATLEN_1                   \
                   | SPI_I2SCFGR_CHLEN )

/* ---- Alternate-function pins (spi_i2s.md FUN_080320fc), set before I2SE ---- */
static void i2s_pins_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA
                           | LL_AHB1_GRP1_PERIPH_GPIOB
                           | LL_AHB1_GRP1_PERIPH_GPIOC);

    /* PA15 AF6 */
    M_AF(GPIOA, 15); LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_15, LL_GPIO_AF_6);
    /* PC10 AF6, PC11 AF5, PC12 AF6 */
    M_AF(GPIOC, 10); LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_10, LL_GPIO_AF_6);
    M_AF(GPIOC, 11); LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_11, LL_GPIO_AF_5);
    M_AF(GPIOC, 12); LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_12, LL_GPIO_AF_6);
    /* PB12 AF5, PB13 AF5, PB14 AF6, PB15 AF5 */
    M_AF(GPIOB, 12); LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_12, LL_GPIO_AF_5);
    M_AF(GPIOB, 13); LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_13, LL_GPIO_AF_5);
    M_AF(GPIOB, 14); LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_14, LL_GPIO_AF_6);
    M_AF(GPIOB, 15); LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_15, LL_GPIO_AF_5);
}

/* ---- One DMA1 stream: 16-bit periph+mem, MINC, circular double-buffer ---- */
static void dma_cfg(DMA_Stream_TypeDef *st, uint32_t chsel, volatile void *periph,
                    void *m0, void *m1, int dir_m2p, int tcie)
{
    st->CR &= ~DMA_SxCR_EN;
    while (st->CR & DMA_SxCR_EN)
        ;
    st->PAR  = (uint32_t)periph;
    st->M0AR = (uint32_t)m0;
    st->M1AR = (uint32_t)m1;
    st->NDTR = I2S_BANK_HALFWORDS;
    st->CR = (chsel << DMA_SxCR_CHSEL_Pos)
           | (1u << DMA_SxCR_PSIZE_Pos)      /* 16-bit periph */
           | (1u << DMA_SxCR_MSIZE_Pos)      /* 16-bit memory */
           | DMA_SxCR_MINC
           | DMA_SxCR_CIRC
           | DMA_SxCR_DBM                    /* double-buffer */
           | (dir_m2p ? DMA_SxCR_DIR_0 : 0u) /* 01 = mem->periph */
           | (tcie ? DMA_SxCR_TCIE : 0u);
    /* EN left clear; i2s_start() enables all four together. */
}

void i2s_config(void)
{
    i2s_pins_init();

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2 | LL_APB1_GRP1_PERIPH_SPI3);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    /* Receive blocks: slave receive + RX DMA request. */
    SPI2->I2SCFGR = I2SCFGR_RX;  SPI2->CR2 = SPI_CR2_RXDMAEN;
    SPI3->I2SCFGR = I2SCFGR_RX;  SPI3->CR2 = SPI_CR2_RXDMAEN;
    /* Extension blocks: slave transmit + TX DMA request. */
    I2S2ext->I2SCFGR = I2SCFGR_TX;  I2S2ext->CR2 = SPI_CR2_TXDMAEN;
    I2S3ext->I2SCFGR = I2SCFGR_TX;  I2S3ext->CR2 = SPI_CR2_TXDMAEN;

    /* Four DMA1 streams (dma.md). p->m RX on S0/S3, m->p TX on S4/S5. Only the
     * SPI2/I2S2 RX stream (S3) carries the transfer-complete interrupt. */
    dma_cfg(DMA1_Stream0, 0u, &SPI3->DR,    rx_iq_m0,    rx_iq_m1,    0, 0); /* SPI3 RX  */
    dma_cfg(DMA1_Stream3, 0u, &SPI2->DR,    rx_audio_m0, rx_audio_m1, 0, 1); /* SPI2 RX  */
    dma_cfg(DMA1_Stream4, 2u, &I2S2ext->DR, tx_audio_m0, tx_audio_m1, 1, 0); /* I2S2ext  */
    dma_cfg(DMA1_Stream5, 2u, &I2S3ext->DR, tx_iq_m0,    tx_iq_m1,    1, 0); /* I2S3ext  */

    /* TX banks start zero so the transmit pairs are electrically inactive. */
    for (unsigned i = 0; i < I2S_BANK_HALFWORDS; i++) {
        tx_audio_m0[i] = tx_audio_m1[i] = 0u;
        tx_iq_m0[i]    = tx_iq_m1[i]    = 0u;
    }
}

/* Enable one full-duplex I2S slave pair synchronized to a WS (LRCLK) rising
 * edge. The codec is already free-running (its clocks came up before i2s_config),
 * so setting I2SE blindly latches the slave at whatever WS phase is present at
 * that instant -- and that instant shifts with core clock, so at 168 MHz the
 * slaves start mid-frame -> frame error (FRE) and 16-bit misalignment (benign-
 * looking at 250 MHz only by luck). Waiting for a WS low->high edge first (start
 * of the WS-high / right-channel phase, a full half-frame before the falling
 * edge the Philips slave aligns to) makes the enable deterministic at any clock.
 * The ext (TX) block is enabled immediately before its main (RX) block so both
 * latch the same frame. ws_mask = the WS pin on ws_idr (I2S2 WS=PB12, I2S3=PA15). */
static void i2s_enable_pair_synced(volatile uint32_t *ws_idr, uint32_t ws_mask,
                                   SPI_TypeDef *ext, SPI_TypeDef *main_)
{
    (void)main_->SR; (void)ext->SR;                 /* clear any stale FRE/OVR */
    uint32_t t;
    for (t = 0; (*ws_idr & ws_mask)  && t < 2000000u; t++) { }   /* until WS low  */
    for (t = 0; !(*ws_idr & ws_mask) && t < 2000000u; t++) { }   /* WS rising edge */
    ext->I2SCFGR  |= SPI_I2SCFGR_I2SE;
    main_->I2SCFGR |= SPI_I2SCFGR_I2SE;
}

void i2s_start(void)
{
    /* Enable all four DMA streams first (startup.md step 4). */
    DMA1_Stream0->CR |= DMA_SxCR_EN;
    DMA1_Stream3->CR |= DMA_SxCR_EN;
    DMA1_Stream4->CR |= DMA_SxCR_EN;
    DMA1_Stream5->CR |= DMA_SxCR_EN;

    NVIC_SetPriority(DMA1_Stream3_IRQn, 5);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);

    /* Then enable the I2S RX/TX blocks (startup.md step 5), each pair synced to
     * its own WS edge so the slaves align regardless of core clock. */
    i2s_enable_pair_synced(&GPIOB->IDR, (1u << 12), I2S2ext, SPI2);  /* audio */
    i2s_enable_pair_synced(&GPIOA->IDR, (1u << 15), I2S3ext, SPI3);  /* I/Q   */
}

void i2s_reconstruct_frame(const uint16_t *hw4, int32_t *ch0, int32_t *ch1)
{
    *ch0 = (int32_t)(((uint32_t)hw4[0] << 16) | hw4[1]);
    *ch1 = (int32_t)(((uint32_t)hw4[2] << 16) | hw4[3]);
}

void i2s_process_banks(const uint16_t *rx_audio, const uint16_t *rx_iq,
                       uint16_t *tx_audio, uint16_t *tx_iq)
{
    /* RX: reconstruct signed-32 per channel (DSP-less -> keep the last frame). */
    int32_t a0, a1, q0, q1;
    for (unsigned f = 0; f < I2S_BANK_FRAMES; f++) {
        i2s_reconstruct_frame(&rx_audio[4u * f], &a0, &a1);
        i2s_reconstruct_frame(&rx_iq[4u * f],    &q0, &q1);
    }
    i2s_last_audio[0] = a0; i2s_last_audio[1] = a1;
    i2s_last_iq[0]    = q0; i2s_last_iq[1]    = q1;

    /* TX: zero-fill both selected banks -> transmit pairs stay inactive. */
    for (unsigned i = 0; i < I2S_BANK_HALFWORDS; i++) {
        tx_audio[i] = 0u;
        tx_iq[i]    = 0u;
    }

    i2s_bank_count++;
}

/* Weak DSP hook. The default is the Phase-2b DSP-less service (reconstruct RX,
 * zero-fill TX). A phase that does real receive DSP (Phase 3) overrides this
 * with a strong definition of the same name; common/i2s.c and the earlier
 * phases are then unaffected. Called from the Stream3 ISR with the completed
 * RX banks and the selected (about-to-transmit) TX banks. */
__attribute__((weak)) void i2s_dsp(const uint16_t *rx_audio, const uint16_t *rx_iq,
                                   uint16_t *tx_audio, uint16_t *tx_iq)
{
    i2s_process_banks(rx_audio, rx_iq, tx_audio, tx_iq);
}

void i2s_on_bank_complete(void)
{
    /* Bank ownership: RX pair from Stream3 CT, TX pair from Stream4 CT
     * (dma.md). The finished bank is the one not currently targeted. */
    int rx_ct = !!(DMA1_Stream3->CR & DMA_SxCR_CT);
    int tx_ct = !!(DMA1_Stream4->CR & DMA_SxCR_CT);
    i2s_rx_audio = i2s_select_bank(rx_ct, (uint16_t *)DMA1_Stream3->M0AR,
                                          (uint16_t *)DMA1_Stream3->M1AR);
    i2s_rx_iq    = i2s_select_bank(rx_ct, (uint16_t *)DMA1_Stream0->M0AR,
                                          (uint16_t *)DMA1_Stream0->M1AR);
    i2s_tx_audio = i2s_select_bank(tx_ct, (uint16_t *)DMA1_Stream4->M0AR,
                                          (uint16_t *)DMA1_Stream4->M1AR);
    i2s_tx_iq    = i2s_select_bank(tx_ct, (uint16_t *)DMA1_Stream5->M0AR,
                                          (uint16_t *)DMA1_Stream5->M1AR);

    i2s_dsp((const uint16_t *)i2s_rx_audio, (const uint16_t *)i2s_rx_iq,
            (uint16_t *)i2s_tx_audio, (uint16_t *)i2s_tx_iq);
}

/* Vector idx30 / IRQn 14. Clears the Stream3 transfer-complete flag, then runs
 * the (DSP-less) bank service shared with the self-test. */
void DMA1_Stream3_IRQHandler(void)
{
    i2s_isr_entries++;
    if (DMA1->LISR & DMA_LISR_TCIF3) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF3;
        i2s_on_bank_complete();
    }
}
