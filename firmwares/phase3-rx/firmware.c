/*
 * x6100-bbfw -- Phase 3: receive-only radio (AM on 1410 kHz, WDOV)
 * ================================================================
 * Builds on the proven Phase 2b-HW bring-up (clocks, I2C, VersaClock OUT0 =
 * 26 MHz codec MCLK, TLV320/CS4272 codec, four-stream I2S transport) and adds
 * the three receive-only pieces from docs/{frequency_control,rf_switching,
 * rx_pipeline}.md:
 *
 *   1. LO:  retune VersaClock OUT1 to 4 x Ftuned (QSD divides by 4), computed
 *           at runtime from the public 5P49V60 register spec -- vc_set_out1_hz().
 *   2. RF:  primary band index (direct-GPIO PD9/10/11/13) + secondary sub-band
 *           index on the 24-bit SPI6 image (PG13 SCK / PG14 MOSI / PG12 latch).
 *   3. DSP: AM envelope demodulator on the I/Q banks (rx_dsp.c), audio out the
 *           I2S2ext pair -> TLV320 DAC -> headphone/speaker.
 *
 * SAFETY: receive-only. The PA tuple stays in its RX/disabled state, the
 * charger stays off, and the I/Q transmit pair (I2S3ext) stays zero-filled.
 * No path here asserts the TX tuple (implementation_readiness.md Phase 3).
 *
 * Target: WDOV 1410 kHz -- strong local AM broadcast carrier, on 24/7. AM is
 * demodulated as sqrt(I^2+Q^2), which is invariant to I/Q swap/sign, so this
 * first receiver does not depend on the still-unverified converter polarity.
 *
 * (Original Phase 2a/2b bring-up body follows.)
 * -----------------------------------------------------------------------
 * The analog/I2C part of Phase 2 (x6100-base/docs/implementation_readiness.md).
 *
 * This firmware:
 *   - initializes ADC1 (ch0/1/2) + ADC2 (ch6/7/14) scan groups with DMA2
 *     (analog.md), and the internal DAC with the DAC2 BAND_V path;
 *   - reads + decodes MAX17205 telemetry: Batt, RepSOC, Temp, AvgCurrent
 *     (i2c.md / charger.md scale factors);
 *   - releases the codec reset (PE15) and runs the exact TLV320AIC3204 startup
 *     (clock helper FUN_0802f0ec + the 58-step sequence), the 0x4A six-write
 *     companion, and the CS4272 probe/init with the 0x4C fallback branch
 *     (i2c.md, tlv320aic3204_startup_sequence.csv, i2c2_aux_startup_sequences.csv).
 *
 * Emulator validation (deterministic, model-independent):
 *   - ADC2 external-voltage scaling + IIR filter: numeric self-test
 *     (y_ss = mean(samples); mV = getter0 * 4.835165);
 *   - MAX17205 engineering-unit decode;
 *   - codec/converter register-write read-back from the I2C device models.
 * Live ADC acquisition and I2S/BCLK are hardware (or Phase 2b) concerns.
 *
 * PA and charger stay disabled. The codec leaves its outputs muted, so bringing
 * it out of reset is safe.
 */
#include <stdint.h>

#include "bsp.h"
#include "i2c.h"
#include "i2s.h"
#include "rx_dsp.h"

#define FW_VERSION "x6100-bbfw phase3-rx 2026-10-31"

/* Tuned RX frequency. WDOV = 1410 kHz AM broadcast. AM (mode 6) applies a
 * -100 Hz LO offset (frequency_control.md); negligible for envelope detection
 * but included for faithfulness. */
#define RX_TUNED_HZ   1410000u
#define AM_LO_OFFSET  (-100)      /* mode-6 adjustment (frequency_control.md) */

/* Default AGC: ON at boot so loudness is consistent across stations and VOLUME
 * sits near unity (paired with the +9 dB HP driver, see codec_unmute_hp). Bench
 * settings from the phase3-rx experiment. Override/disable live via CMD_AGC. */
#define RX_AGC_DEF_TARGET_CDB  (-1200)   /* -12.00 dBFS target */
#define RX_AGC_DEF_ATK_MS      70u
#define RX_AGC_DEF_DCY_MS      250u
#define RX_AGC_DEF_HANG_MS     0u

#define ADDR_MAX17205   0x36u
#define ADDR_CODEC      0x18u
#define ADDR_CS4272     0x10u
#define ADDR_AUX4A      0x4Au
#define ADDR_AUX4C      0x4Cu
#define ADDR_VERSACLOCK 0x6Au

/* Phase-2 bring-up diagnostics (VersaClock VCO-lock readback, codec page-0
 * clock-register dump, I2S clock sniff, per-second SR/NDTR/ISR dump). OFF by
 * default now that the transport is proven; re-enable quickly by uncommenting
 * the define below or building with -DPHASE2_DIAG. */
/* #define PHASE2_DIAG 1 */

/* ======================================================================== *
 *  ADC (analog.md): two independent 3-channel scan groups with DMA2.
 *  ADC1 ch0/1/2 -> DMA2 Stream0 ch0 (3 halfwords); fast RF measurements.
 *  ADC2 ch6/7/14 -> DMA2 Stream2 ch1 (15 halfwords = 5 scans); telemetry.
 * ======================================================================== */
static volatile uint16_t adc1_buf[3];
static volatile uint16_t adc2_buf[15];

/* ADC2 telemetry IIR state (analog.md FUN_0803082c): y = 0.92 y + 0.016*sum5. */
static float adc2_y[3];

static void adc_dma_stream_init(DMA_Stream_TypeDef *st, uint32_t chsel,
                                volatile void *periph, volatile void *mem,
                                uint32_t count)
{
    st->CR &= ~DMA_SxCR_EN;
    while (st->CR & DMA_SxCR_EN)
        ;
    st->PAR  = (uint32_t)periph;
    st->M0AR = (uint32_t)mem;
    st->NDTR = count;
    /* channel select, 16-bit periph+mem, MINC, circular, periph->mem. */
    st->CR = (chsel << DMA_SxCR_CHSEL_Pos)
           | (1u << DMA_SxCR_PSIZE_Pos)      /* 16-bit */
           | (1u << DMA_SxCR_MSIZE_Pos)      /* 16-bit */
           | DMA_SxCR_MINC
           | DMA_SxCR_CIRC;
    st->CR |= DMA_SxCR_EN;
}

static void adc_init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1 | LL_APB2_GRP1_PERIPH_ADC2);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

    /* ADC prescaler /4: PCLK2 125 MHz -> 31.25 MHz (< 36 MHz max). */
    ADC123_COMMON->CCR = (1u << ADC_CCR_ADCPRE_Pos);

    /* ADC1: scan, continuous, DMA + DDS; regular group ch0,ch1,ch2. */
    ADC1->CR1 = ADC_CR1_SCAN;
    ADC1->CR2 = ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS;
    ADC1->SMPR2 = (3u << 0) | (3u << 3) | (3u << 6);          /* ch0/1/2 15c */
    ADC1->SQR1 = (2u << ADC_SQR1_L_Pos);                      /* 3 conv */
    ADC1->SQR3 = (0u << 0) | (1u << 5) | (2u << 10);          /* ch0,1,2 */

    /* ADC2: scan, continuous, DMA + DDS; regular group ch6,ch7,ch14. */
    ADC2->CR1 = ADC_CR1_SCAN;
    ADC2->CR2 = ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS;
    ADC2->SMPR2 = (3u << 18) | (3u << 21);                    /* ch6,ch7 15c */
    ADC2->SMPR1 = (3u << 12);                                 /* ch14 15c */
    ADC2->SQR1 = (2u << ADC_SQR1_L_Pos);                      /* 3 conv */
    ADC2->SQR3 = (6u << 0) | (7u << 5) | (14u << 10);         /* ch6,7,14 */

    adc_dma_stream_init(DMA2_Stream0, 0u, &ADC1->DR, adc1_buf, 3u);
    adc_dma_stream_init(DMA2_Stream2, 1u, &ADC2->DR, adc2_buf, 15u);

    ADC1->CR2 |= ADC_CR2_ADON;
    ADC2->CR2 |= ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    ADC2->CR2 |= ADC_CR2_SWSTART;
}

/* One telemetry filter step over the 5-set interleaved ADC2 buffer. */
static void adc2_filter_step(const uint16_t *buf)
{
    for (int ch = 0; ch < 3; ch++) {
        uint32_t sum = (uint32_t)buf[ch] + buf[ch + 3] + buf[ch + 6]
                     + buf[ch + 9] + buf[ch + 12];
        adc2_y[ch] = 0.92f * adc2_y[ch] + 0.016f * (float)sum;
    }
}

/* External-supply voltage from ADC2 getter 0 (analog.md). */
static float external_voltage_mv(void)
{
    return adc2_y[0] * 4.835165f;
}

/* ======================================================================== *
 *  Internal DAC (analog.md FUN_08030aa0): both channels zeroed + enabled;
 *  DAC2/PA5 is the ACC BAND_V staircase (frequency-dependent in Phase 3).
 * ======================================================================== */
static void dac_init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);
    DAC->DHR12R1 = 0u;
    DAC->DHR12R2 = 0u;
    DAC->CR = DAC_CR_EN1 | DAC_CR_EN2;
}

/* ======================================================================== *
 *  MAX17205 telemetry decode (i2c.md FUN_08031e1c; charger.md scales).
 * ======================================================================== */
static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

/* ======================================================================== *
 *  VersaClock 5P49V60 (I2C1 0x6A): the sample-clock source. OUT0 = 26 MHz ->
 *  TLV320 MCLK = codec PLL reference; without it the codec PLL never locks, no
 *  BCLK/WCLK is generated, and the I2S DMA banks never advance. PE7 = enable.
 *  Public-spec reimplementation (frequency_control.md): only non-default regs
 *  plus the 0x76,0x43/0x63 calibration kicks. Ported from the experiment
 *  firmwares/phase2b-sample-transport-250. Needs PE14 high (I2C1 enable).
 * ======================================================================== */
static const uint8_t VC_CFG[] = {
    5, 0x00, 0x61, 0x03, 0xFF, 0x00,
    2, 0x0B, 0xC0,
    3, 0x10, 0xA0, 0xCC,
    2, 0x15, 0x0A,
    3, 0x17, 0x06, 0xF0,
    5, 0x1C, 0x9F, 0xFF, 0xE8, 0x80,
    5, 0x22, 0x01, 0x1F, 0x09, 0x28,
    2, 0x2A, 0x04,
    3, 0x2D, 0x01, 0x90,
    2, 0x30, 0x80,
    2, 0x3A, 0x04,
    4, 0x3F, 0x39, 0x80, 0x00,
    2, 0x4A, 0x04,
    2, 0x4F, 0x38,
    3, 0x51, 0x81, 0x02,
    2, 0x5A, 0x04,
    14, 0x5D, 0x2D, 0x10, 0x20,
        0x38, 0x05, 0xBB, 0x04, 0xBB, 0x04, 0x38, 0x05, 0xDF, 0x4C,
    0
};

static void vc_write_seq(const uint8_t *seq)
{
    while (*seq) { uint8_t n = *seq++; i2c_write_raw(I2C1, ADDR_VERSACLOCK, seq, n); seq += n; }
}

#ifdef PHASE2_DIAG
static uint8_t vc_read8(uint8_t reg)
{
    uint8_t v = 0xEE;
    i2c_read(I2C1, ADDR_VERSACLOCK, reg, &v, 1);
    return v;
}
/* APLL/VCO lock + calibration status via the UNDOCUMENTED 0x90-0x9F "Factory
 * Use" registers (Renesas FAE, community thread "5P49V60 OUT0 dropout"):
 * 0x9D[7]=PLL_Lock, 0x99[2]=Calibration_Done, 0x99[7:3]=VCO band, 0x6F=Lock
 * bypass. Ported from the experiment firmwares/versaclock-alive-check. An
 * unlocked VCO = no clean 26 MHz on OUT0 = codec PLL never locks = banks=0.
 * Returns PLL_Lock. */
static int vc_status(const char *tag)
{
    uint8_t r99 = vc_read8(0x99), r9D = vc_read8(0x9D), r6F = vc_read8(0x6F);
    int locked = (r9D >> 7) & 1, cal_done = (r99 >> 2) & 1, band = (r99 >> 3) & 0x1F;
    bsp_uprintf("  VC %s PLL_Lock=%d Cal_Done=%d band=%d [0x99=0x%02X 0x9D=0x%02X 0x6F=0x%02X]\r\n",
                tag, locked, cal_done, band, r99, r9D, r6F);
    return locked;
}
#endif /* PHASE2_DIAG */

/* Returns 1 if 0x6A ACKed its address on I2C1 (validates PE14 I2C1-enable).
 * Programs the VC and reads VCO-lock status. On hardware the VCO locks from the
 * OEM 0x76 global-reset kicks alone, so the explicit Renesas 0x1C[7] calibration
 * is NOT run (kept out; firmwares/versaclock-alive-check has it if ever needed). */
static int versaclock_init(void)
{
    static const uint8_t k43[] = {0x76, 0x43}, k63[] = {0x76, 0x63};
    SET_LO(GPIOE, 7); bsp_delay_ms(100);     /* PE7: VC reset  */
    SET_HI(GPIOE, 7); bsp_delay_ms(100);     /* PE7: VC enable */
    int present = i2c_probe(I2C1, ADDR_VERSACLOCK);
    vc_write_seq(VC_CFG);                         bsp_delay_ms(10);
    i2c_write_raw(I2C1, ADDR_VERSACLOCK, k43, 2); bsp_delay_ms(10);
    i2c_write_raw(I2C1, ADDR_VERSACLOCK, k63, 2); bsp_delay_ms(10);
    i2c_write_raw(I2C1, ADDR_VERSACLOCK, k43, 2); bsp_delay_ms(10);
    i2c_write_raw(I2C1, ADDR_VERSACLOCK, k63, 2); bsp_delay_ms(10);

#ifdef PHASE2_DIAG
    int locked = vc_status("after program:");
    bsp_uprintf("  VC VCO %s\r\n",
                locked ? "LOCKED -> OUT0 should clock 26 MHz"
                       : "NOT LOCKED (ref/VCO problem; see versaclock-alive-check cal)");
#endif
    return present;
}

/* ======================================================================== *
 *  LO tuning: VersaClock OUT1 = 4 x Ftuned  (frequency_control.md)
 *  ------------------------------------------------------------------------
 *  The receiver is a quadrature-sampling detector whose switching clock runs
 *  at 4 x Ftuned; OUT1 therefore carries 4 x Ftuned. OUT1 is the FOD1
 *  fractional output divider (5P49V60 programming guide):
 *      FOUT = (FVCO/2) / N,   N = OD1_intdiv + OD1_offset / 2^24
 *  The APLL/VCO is fixed by the OEM VC_CFG feedback divider: FB_intdiv = 0x06F
 *  = 111, fractional = 0, pre-divider bypassed (0x16 default) -> with the
 *  26 MHz reference, FVCO = 26e6 * 111 = 2886 MHz and FVCO/2 = 1443 MHz. (The
 *  VC_CFG already programs OUT1 to ~57 MHz = 4 x 14.27 MHz; this retunes it.)
 *
 *  Register layout (programming guide tables 51/52, 59-62), implemented from
 *  the public spec per the doc's instruction (NOT copied from OEM output):
 *      0x2D = OD1_intdiv[11:4]            0x2E[7:4] = OD1_intdiv[3:0]
 *      0x22 = OD1_offset[29:22]  0x23 = [21:14]  0x24 = [13:6]
 *      0x25[7:2] = OD1_offset[5:0], 0x25[1] = ssce (0 = no spread)
 *  Commit is the cached 3-byte integer group then the 5-byte fractional group
 *  (FUN_08033514). Sanity: 5.640 MHz -> N=255.851 -> intdiv=0xFF, off=0xD9DF52
 *  -> {0x2D=0x0F,0x2E=0xF0} {0x22=0x03,0x23=0x67,0x24=0x7D,0x25=0x48}.
 * ======================================================================== */
#define VC_FVCO_DIV2_HZ  1443000000.0   /* (26 MHz * 111) / 2 */

static void vc_set_out1_hz(uint32_t fout_hz)
{
    double n = VC_FVCO_DIV2_HZ / (double)fout_hz;     /* total divide ratio   */
    uint32_t intdiv = (uint32_t)n;                    /* 12-bit integer part  */
    double frac = n - (double)intdiv;                 /* 0 <= frac < 1        */
    uint32_t off = (uint32_t)(frac * 16777216.0 + 0.5); /* OD1_offset = frac*2^24 */

    uint8_t ig[3] = { 0x2D,
                      (uint8_t)((intdiv >> 4) & 0xFF),      /* [11:4] */
                      (uint8_t)((intdiv & 0x0F) << 4) };    /* [3:0] in D7:4 */
    uint8_t fg[5] = { 0x22,
                      (uint8_t)((off >> 22) & 0xFF),
                      (uint8_t)((off >> 14) & 0xFF),
                      (uint8_t)((off >>  6) & 0xFF),
                      (uint8_t)((off & 0x3F) << 2) };        /* D7:2 = [5:0], ssce=0 */
    i2c_write_raw(I2C1, ADDR_VERSACLOCK, ig, sizeof ig);
    i2c_write_raw(I2C1, ADDR_VERSACLOCK, fg, sizeof fg);
    bsp_uprintf("  LO: OUT1=%lu Hz  N=%lu.%06lu  intdiv=0x%03lX off=0x%06lX\r\n"
                "      regs 0x2D=%02X 0x2E=%02X | 0x22=%02X 0x23=%02X 0x24=%02X 0x25=%02X\r\n",
                (unsigned long)fout_hz, (unsigned long)intdiv,
                (unsigned long)((n - (double)intdiv) * 1000000.0 + 0.5),
                (unsigned long)intdiv, (unsigned long)off,
                ig[1], ig[2], fg[1], fg[2], fg[3], fg[4]);
}

/* ======================================================================== *
 *  RF band switching (rf_switching.md). This board is the DIRECT-GPIO variant
 *  (Phase 1: PCA9535 0x20 absent), so the primary RF index is four binary GPIO
 *  outputs; the secondary sub-band index is one bit of the 24-bit SPI6 image.
 * ======================================================================== */

/* Primary index -> frequency region (frequency_control.md). */
static int rf_primary_index(uint32_t hz)
{
    if (hz < 1500000u  || hz > 55000000u) return 0;
    if (hz <= 2200000u)  return 1;
    if (hz <= 3800000u)  return 2;
    if (hz <= 5500000u)  return 3;
    if (hz <= 7500000u)  return 4;
    if (hz <= 13500000u) return 5;
    if (hz <= 21000000u) return 6;
    if (hz <= 33000000u) return 7;
    if (hz <= 50000000u) return 8;
    return 9;
}

/* Secondary sub-band index -> shared-image bit (rf_switching.md). */
static int rf_secondary_index(uint32_t hz)
{
    if (hz <= 2000000u)  return 0;   /* bit 13 */
    if (hz <= 4000000u)  return 1;   /* bit 12 */
    if (hz <= 8000000u)  return 2;   /* bit 11 */
    if (hz <= 15000000u) return 3;   /* bit 10 */
    if (hz <= 22000000u) return 4;   /* bit  9 */
    if (hz <= 30000000u) return 5;   /* bit  8 */
    return 6;                        /* bit  7 */
}

/* Direct-GPIO primary index: weight 1=PD9, 2=PD10, 4=PD11, 8=PD13 (binary). */
static void rf_apply_primary(int idx)
{
    (idx & 1) ? SET_HI(GPIOD, 9)  : SET_LO(GPIOD, 9);
    (idx & 2) ? SET_HI(GPIOD, 10) : SET_LO(GPIOD, 10);
    (idx & 4) ? SET_HI(GPIOD, 11) : SET_LO(GPIOD, 11);
    (idx & 8) ? SET_HI(GPIOD, 13) : SET_LO(GPIOD, 13);
}

/* SPI6 shared 24-bit output image (atu.md). PG13=SCK AF5, PG14=MOSI AF5,
 * PG12=latch GPIO. Receive-only: the ATU and other owners are zero, so the
 * image is just the secondary RF sub-band bit. */
static uint32_t rf_image;

static void spi6_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOG);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI6);

    M_OUT(GPIOG, 12); M_PP(GPIOG, 12); SPD_HI(GPIOG, 12); SET_LO(GPIOG, 12);
    M_AF(GPIOG, 13); SPD_HI(GPIOG, 13); LL_GPIO_SetAFPin_8_15(GPIOG, LL_GPIO_PIN_13, LL_GPIO_AF_5);
    M_AF(GPIOG, 14); SPD_HI(GPIOG, 14); LL_GPIO_SetAFPin_8_15(GPIOG, LL_GPIO_PIN_14, LL_GPIO_AF_5);

    /* SPI6 master, 8-bit, software NSS, TX only. PCLK2 (125/84 MHz) / 256 keeps
     * the serial clock conservative for the board's relay-driver chain. */
    SPI6->CR1 = 0;
    SPI6->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI
              | (7u << SPI_CR1_BR_Pos) | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE;
    SPI6->CR1 |= SPI_CR1_SPE;
}

static void spi6_send_byte(uint8_t b)
{
    while (!(SPI6->SR & SPI_SR_TXE))
        ;
    *(volatile uint8_t *)&SPI6->DR = b;
}

/* Commit the 24-bit image high-byte-first while PG12 is high, then latch low
 * (atu.md FUN_08030144). */
static void rf_commit_image(uint32_t img)
{
    SET_HI(GPIOG, 12);
    spi6_send_byte((uint8_t)(img >> 16));
    spi6_send_byte((uint8_t)(img >> 8));
    spi6_send_byte((uint8_t)(img));
    while (SPI6->SR & SPI_SR_BSY)
        ;
    SET_LO(GPIOG, 12);
}

/* Apply both RF indices for a tuned frequency, preserving unrelated image bits
 * (clear only bits 7..13 then set the one secondary bit). */
static void rf_switch_tune(uint32_t hz)
{
    int pidx = rf_primary_index(hz);
    int sidx = rf_secondary_index(hz);
    int sbit = 13 - sidx;                      /* idx0->bit13 ... idx6->bit7 */
    rf_apply_primary(pidx);
    rf_image = (rf_image & ~(0x3F80u)) | (1u << sbit);   /* 0x3F80 = bits 7..13 */
    rf_commit_image(rf_image);
    bsp_uprintf("  RF: primary idx=%d (PD9/10/11/13) secondary idx=%d (image bit %d) "
                "img=0x%06lX\r\n", pidx, sidx, sbit, (unsigned long)rf_image);
}

/* Frequency-dependent BAND_V (DAC2/PA5) staircase code (frequency_control.md). */
static uint16_t band_v_code(uint32_t hz)
{
    if (hz <=  2000000u) return 0x11Du;
    if (hz <=  5000000u) return 0x23Au;
    if (hz <=  7000000u) return 0x358u;
    if (hz <= 10000000u) return 0x475u;
    if (hz <= 14000000u) return 0x593u;
    if (hz <= 18000000u) return 0x6B0u;
    if (hz <= 21000000u) return 0x7CDu;
    if (hz <= 24000000u) return 0x8EBu;
    if (hz <= 28000000u) return 0xA08u;
    if (hz <= 50000000u) return 0xB26u;
    return 0xC43u;
}

/* --------------------------------------------------------------------------
 * Software serial cf88 shift chain (gpio.md "Serial GPIO shift chains"; OEM
 * FUN_0802cf88). Six bits of `word` are shifted MSB-first out PC5 (data), each
 * clocked by a PB0 pulse, then PB1 is pulsed to latch. PC5/PB0/PB1 are already
 * push-pull outputs (bsp.c).
 * This chain is the TX POWER attenuator, NOT an RX filter (see rf_chain_code()
 * below for the decode). The RX front-end enable is PG2 (common/bsp.c); that --
 * not this chain -- is what previously left RX at the noise floor. Retained so
 * the SC6 debug opcode can still drive it, and so apply_tune() holds it at the
 * OEM power-on word 0x0E. Driving it does not affect RX.
 * -------------------------------------------------------------------------- */
static inline void chain_delay(void)
{
    for (volatile int i = 0; i < 8; i++)   /* ~ tens of ns at 250 MHz */
        ;
}

static void rf_chain6(uint8_t word)
{
    for (int bit = 5; bit >= 0; bit--) {               /* MSB (bit 5) first */
        if ((word >> bit) & 1u) SET_HI(GPIOC, 5);      /* PC5 = data bit    */
        else                    SET_LO(GPIOC, 5);
        chain_delay();
        SET_HI(GPIOB, 0); chain_delay();               /* PB0 clock high    */
        SET_LO(GPIOB, 0); chain_delay();               /* PB0 clock low     */
    }
    SET_HI(GPIOB, 1); chain_delay();                   /* PB1 latch pulse   */
    SET_LO(GPIOB, 1);
}

/* SECOND shift chain: 8 bits shifted MSB-first out PC5 (data) with a PB0 clock
 * pulse each, PG6 held low during the shift then raised to latch (OEM
 * FUN_0802d15c / FUN_0802d310). Shares PC5/PB0 with rf_chain6() but latches on
 * PG6 instead of PB1.
 *
 * NOTE (oem-rftrace, 2026-11-02): stock 1.1.9 NEVER drives this chain. Its only
 * caller (startup init FUN_0802a5d0) gates the d15c + three d310 calls behind
 * FUN_0803147c(), which is `return 1`, so the block is skipped and the
 * PG6-latched register is left in its power-on state. A hardware trace showed
 * zero d15c/d310 calls across boot and tuning. phase3-rx therefore does NOT
 * program it either (an earlier rf_chain_startup(0x1A) here was a stray write
 * the OEM never makes). This routine is retained only for the SC8PG6 debug
 * opcode, so the PG6 register can still be swept by hand if ever needed.
 * PC5/PB0/PG6 are already push-pull outputs (bsp.c). */
static void rf_chain_pg6(uint8_t word)
{
    SET_LO(GPIOG, 6); chain_delay();                   /* PG6 latch low      */
    for (int bit = 7; bit >= 0; bit--) {               /* MSB first          */
        if ((word >> bit) & 1u) SET_HI(GPIOC, 5);      /* PC5 = data bit     */
        else                    SET_LO(GPIOC, 5);
        chain_delay();
        SET_HI(GPIOB, 0); chain_delay();               /* PB0 clock high     */
        SET_LO(GPIOB, 0); chain_delay();               /* PB0 clock low      */
    }
    SET_HI(GPIOG, 6);                                  /* PG6 latch pulse hi */
}

/* 6-bit cf88 serial-chain word. IMPORTANT: this is NOT an RX band filter -- it
 * is the TX POWER attenuator. In stock 1.1.9 the only per-tune caller of
 * FUN_0802cf88 (FUN_0802af0c) computes the word from the TX power setting, not
 * frequency: word = 0x64 - power (power<0x25 -> 0x3F; 0x25..0x64 -> 0x64-power;
 * >=0x65 -> 0). The earlier "0x25 @ 1410 / 0x3F @ 9.4 MHz" captures only LOOKED
 * band-indexed because the OEM reloads a per-band default TX power on each band
 * change. Confirmed in hardware: sweeping cf88 (SC6 opcode) across 0x00..0x3F
 * changes RX not at all -- PG2 is the RX front-end enable; cf88 drove the TX
 * LED. So for this RX-only firmware the value is electrically irrelevant; we
 * hold it at the OEM power-on word 0x0E (a benign "near-min drive" state) and
 * never key TX. (Full analysis: x6100-base callgraph.csv + FUN_0802af0c.) */
static uint8_t rf_chain_code(uint32_t hz)
{
    (void)hz;                            /* cf88 is TX-power, not frequency */
    return 0x0Eu;                        /* OEM power-on word; RX-irrelevant */
}

/* Live LO offset (ftuned = station + offset) and the last-tuned station, so a
 * CMD_LO_OFFSET change can re-apply the LO at the current frequency. Default is
 * AM_LO_OFFSET (-100 Hz) -- unchanged from before this command existed. */
static volatile int32_t rx_lo_offset_hz = AM_LO_OFFSET;
static uint32_t rx_cur_station_hz = RX_TUNED_HZ;

/* Program LO + RF switching + BAND_V for a station frequency (applies the live
 * LO offset). Used for the initial tune and for every TUNE command. */
static void apply_tune(uint32_t station_hz)
{
    rx_cur_station_hz = station_hz;
    uint32_t ftuned = (uint32_t)((int32_t)station_hz + rx_lo_offset_hz);
    uint8_t  sc6    = rf_chain_code(station_hz);
    rf_chain6(sc6);                           /* cf88 = TX-power attenuator (RX-irrelevant; held 0x0E) */
    vc_set_out1_hz(4u * ftuned);
    rf_switch_tune(ftuned);
    DAC->DHR12R2 = band_v_code(station_hz);   /* BAND_V for this band (was fixed 0x6B0) */
    rx_dsp_flush();                            /* clear DSP histories: no stale transient */
    bsp_uprintf("  RF chain6=0x%02X (serial filter/relay bank)\r\n", sc6);
}

/* ======================================================================== *
 *  Runtime control from the R2/Linux board: TUNE / VOLUME over I2C3.
 *  ------------------------------------------------------------------------
 *  The BASE is the I2C3 slave at 7-bit 0x72 (OAR1 = 0x00E4), PA8 SCL / PC9 SDA
 *  AF4 (i2c.md / linux_hardware_interface.md: Linux reaches it via /dev/i2c-0).
 *  Custom command protocol (we own both ends): the master writes 5 bytes
 *      [opcode, p0, p1, p2, p3]
 *  with the 4 payload bytes little-endian. TUNE (0x01) carries a uint32 Hz;
 *  VOLUME (0x02) carries a float32. The event ISR latches a completed write
 *  into a mailbox; the main loop applies it (TUNE drives I2C1 + SPI6 + delays,
 *  too heavy for interrupt context) and echoes it to USART1.
 *  Initial state: TUNE = 1410000 Hz, VOLUME = 0.0 (silent).
 * ======================================================================== */
#define I2C3_ADDR8   0xE4u     /* 7-bit 0x72 in the OAR1 shifted form */
/* Normal control commands (opcodes 0x01..0x7F). */
#define CMD_TUNE     0x01u     /* u32 Hz  */
#define CMD_VOLUME   0x02u     /* f32     */
#define CMD_AGC      0x03u     /* i16 target(centi-dBFS) + u16 atk(ms) + u16 dcy(ms) [+ u16 hang(ms)];
                                * feed-forward look-ahead AGC (rx_dsp.c). all-zero
                                * (target=atk=dcy=0) => AGC OFF (fixed rx_audio_gain). */
#define CMD_LO_OFFSET 0x04u    /* i32 Hz: LO offset. ftuned = station + offset; LO = 4*ftuned.
                                * Default -100 (LO 100 Hz below dial, carrier at +100 Hz in
                                * baseband). AM envelope is offset-invariant; SSB wants 0. */
#define CMD_MODE     0x05u     /* u8 mode: 1=AM 2=LSB 3=USB. Auto-sets the LO offset for the
                                * mode (AM -> AM_LO_OFFSET, SSB -> 0) and re-tunes. */
#define CMD_LINUX_AUDIO_MULT 0x06u /* f32: HPL / MAIN (R16-capture) output level (left word),
                                    * independent of VOLUME (speaker/right). 0 = HPL silent. */
/* RF-path explorer / debug commands (opcodes 0x81..0xFF). These bypass the
 * frequency->index logic so each RF knob can be swept independently while
 * watching iq_rms, since we cannot scope the hardware. */
#define CMD_RFPRIM   0x81u     /* u32: force primary RF index (0..9) on PD9/10/11/13 */
#define CMD_RFSEC    0x82u     /* u32: force secondary sub-band index (0..6) on SPI6 image */
#define CMD_BANDV    0x83u     /* u32: force BAND_V DAC2 code (12-bit) */
#define CMD_RFIMG    0x84u     /* u32: write a raw 24-bit SPI6 image (probe the 74HC595) */
#define CMD_SC6      0x85u     /* u32: shift a 6-bit word (0..0x3F) through the PC5/PB0/PB1 serial RF chain */
#define CMD_SC8PG6   0x86u     /* u32: shift an 8-bit word through the PC5/PB0/PG6 serial RF chain */
#define CMD_GPIO     0x87u     /* u32: drive ONE pin live. arg nibbles: port(0..6=A..G) |
                                * pin<<4 | mode<<8. mode 0=in 1=in+pd 2=in+pu 3=out-PP-lo
                                * 4=out-PP-hi 5=out-OD-lo 6=out-OD-hi. Sweeps candidate
                                * RX-enable pins (PG8/PE11/PG6/PG2/PG3) the oem-afedump
                                * capture showed the OEM drives but phase3-rx does not. */
#define CMD_CODEC    0x88u     /* u32: write one TLV320AIC3204 register live. arg bytes:
                                * page | reg<<8 | val<<16. Selects the page then writes
                                * reg=val on I2C2 0x18. For tuning HP driver gain/mute
                                * (page1 R0x10/0x11), DAC vol (page0 R0x40/41/42), etc. */
#define CMD_DUMP     0x8Fu     /* u32 (ignored): dump CS4272 + VersaClock + RF state to USART1 */

/* Variable-length command mailbox. The ISR collects a whole I2C write frame
 * into i2c3_rx[], then on STOP snapshots it into cmd_buf[]/cmd_len for the main
 * loop. Commands are dispatched on cmd_buf[0] with per-command length checks
 * (like the experiment's i2c3_service), so each opcode carries as many payload
 * bytes as it needs: 5-byte op+u32 for TUNE/VOLUME/RF/..., 9-byte for AGC. */
#define I2C3_FRAME_MAX 16u
static volatile uint8_t  cmd_pending;
static volatile uint8_t  cmd_len;                 /* bytes latched in cmd_buf */
static volatile uint8_t  cmd_buf[I2C3_FRAME_MAX]; /* [op, payload...], little-endian */
static uint8_t  i2c3_rx[I2C3_FRAME_MAX];
static uint8_t  i2c3_idx;

static void i2c3_slave_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C3);

    /* PA8 = I2C3_SCL, PC9 = I2C3_SDA: AF4, open-drain, pulled up (i2c.md). */
    M_AF(GPIOA, 8); M_OD(GPIOA, 8); PULL_UP(GPIOA, 8); SPD_HI(GPIOA, 8);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_8, LL_GPIO_AF_4);
    M_AF(GPIOC, 9); M_OD(GPIOC, 9); PULL_UP(GPIOC, 9); SPD_HI(GPIOC, 9);
    LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_9, LL_GPIO_AF_4);

    I2C3->CR1 = I2C_CR1_SWRST; I2C3->CR1 = 0;            /* reset peripheral   */
    I2C3->CR2 = (PCLK1_HZ / 1000000u) & I2C_CR2_FREQ;    /* APB1 clock in MHz  */
    I2C3->OAR1 = (1u << 14) | I2C3_ADDR8;                /* 7-bit 0x72, bit14=1 */
    I2C3->OAR2 = 0;
    I2C3->CR1 = I2C_CR1_PE;                              /* enable, then ACK   */
    I2C3->CR1 |= I2C_CR1_ACK;
    I2C3->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN;

    NVIC_SetPriority(I2C3_EV_IRQn, 6);   /* below the DSP ISR; slave stretches  */
    NVIC_EnableIRQ(I2C3_EV_IRQn);
    NVIC_SetPriority(I2C3_ER_IRQn, 6);
    NVIC_EnableIRQ(I2C3_ER_IRQn);
}

/* I2C3 slave event handler (overrides the weak startup.c vector). */
void I2C3_EV_IRQHandler(void)
{
    uint32_t sr1 = I2C3->SR1;
    if (sr1 & I2C_SR1_ADDR) {                 /* address match: read SR1,SR2 */
        (void)I2C3->SR1; (void)I2C3->SR2;
        i2c3_idx = 0;
    }
    if (sr1 & I2C_SR1_RXNE) {                 /* data byte from master       */
        uint8_t b = (uint8_t)I2C3->DR;
        if (i2c3_idx < sizeof i2c3_rx) i2c3_rx[i2c3_idx++] = b;
    }
    if (sr1 & I2C_SR1_TXE)                    /* master read (unused): dummy */
        I2C3->DR = 0x00u;
    if (sr1 & I2C_SR1_STOPF) {                /* STOP: read SR1 then write CR1 */
        (void)I2C3->SR1;
        I2C3->CR1 |= I2C_CR1_PE;
        if (i2c3_idx >= 1u && !cmd_pending) { /* latch the whole frame; len-checked below */
            for (uint8_t k = 0; k < i2c3_idx; k++)
                cmd_buf[k] = i2c3_rx[k];
            cmd_len = i2c3_idx;
            cmd_pending = 1;
        }
        i2c3_idx = 0;
    }
}

void I2C3_ER_IRQHandler(void)
{
    I2C3->SR1 &= ~(I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR);
}

/* Read back converter + synthesizer + RF state to USART1 (no scope available:
 * this is how we confirm the front-end). CS4272 and VersaClock support I2C
 * register reads; i2c_read returns 0 on ACK. */
static void rf_dump(void)
{
    bsp_uprintf("DUMP CS4272(I2C2 0x10):");
    for (uint8_t r = 1; r <= 8; r++) {
        uint8_t v = 0;
        int rc = i2c_read(I2C2, ADDR_CS4272, r, &v, 1);
        bsp_uprintf(" R%02X=%02X%s", r, v, rc ? "!" : "");
    }
    bsp_uprintf("\r\n");

    static const uint8_t vr[] = {0x21, 0x2D, 0x2E, 0x22, 0x23, 0x24, 0x25,
                                 0x60, 0x62, 0x64, 0x66, 0x68, 0x9D};
    bsp_uprintf("DUMP VC(I2C1 0x6A):");
    for (unsigned i = 0; i < sizeof vr; i++) {
        uint8_t v = 0;
        int rc = i2c_read(I2C1, ADDR_VERSACLOCK, vr[i], &v, 1);
        bsp_uprintf(" %02X=%02X%s", vr[i], v, rc ? "!" : "");
    }
    bsp_uprintf("\r\n");
    bsp_uprintf("DUMP BAND_V=0x%03lX RFimg=0x%06lX GPIOD.ODR=0x%08lX (PD9/10/11/13 = primary)\r\n",
                (unsigned long)(DAC->DHR12R2 & 0xFFFu), (unsigned long)rf_image,
                (unsigned long)GPIOD->ODR);
}

static int cwr(uint8_t addr, const uint8_t *buf, uint32_t n);  /* defined below */

/* Apply one pending command (main-loop context) and echo it to USART1. */
static void i2c3_poll_commands(void)
{
    if (!cmd_pending)
        return;
    uint8_t  buf[I2C3_FRAME_MAX];
    uint8_t  n = cmd_len;
    if (n > I2C3_FRAME_MAX) n = I2C3_FRAME_MAX;
    for (uint8_t k = 0; k < n; k++)
        buf[k] = cmd_buf[k];
    cmd_pending = 0;                 /* release the mailbox for the next frame */
    if (n < 1u)
        return;
    uint8_t  op  = buf[0];
    /* AGC is the only variable-length command; every other op is a 5-byte
     * op+u32. Drop short/glitch frames so a stray 1-4 byte write can't e.g.
     * apply_tune(0) (the pre-rewrite code ignored anything under 5 bytes). */
    if (op != CMD_AGC && n < 5u) {
        bsp_uprintf("I2C cmd: short frame op=0x%02X len=%u (ignored)\r\n",
                    op, (unsigned)n);
        return;
    }
    /* Payload as one u32 LE for the op+u32 commands. */
    uint32_t arg = (uint32_t)buf[1] | ((uint32_t)buf[2] << 8)
                 | ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 24);
    switch (op) {
    case CMD_TUNE:
        bsp_uprintf("I2C cmd: TUNE %lu Hz\r\n", (unsigned long)arg);
        apply_tune(arg);
        break;
    case CMD_VOLUME: {
        float v;
        __builtin_memcpy(&v, &arg, sizeof v);
        rx_audio_gain = v;
        bsp_uprintf("I2C cmd: VOLUME %ld/1000\r\n", (long)(v * 1000.0f));
        break;
    }
    case CMD_LINUX_AUDIO_MULT: {
        float v;
        __builtin_memcpy(&v, &arg, sizeof v);
        rx_lin_mult = v;
        bsp_uprintf("I2C cmd: LINUX_AUDIO_MULT %ld/1000 (HPL/MAIN capture level)\r\n",
                    (long)(v * 1000.0f));
        break;
    }
    case CMD_AGC:
        if (n >= 7u) {
            int16_t  target_cdb = (int16_t)((uint16_t)buf[1] | ((uint16_t)buf[2] << 8));
            uint16_t atk_ms = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
            uint16_t dcy_ms = (uint16_t)buf[5] | ((uint16_t)buf[6] << 8);
            uint16_t hang_ms = (n >= 9u) ? (uint16_t)(buf[7] | (buf[8] << 8)) : (uint16_t)0;
            rx_agc_set(target_cdb, atk_ms, dcy_ms, hang_ms);
            if (rx_agc_on)
                bsp_uprintf("I2C cmd: AGC ON target=%ld cdBFS atk=%u dcy=%u hang=%u ms\r\n",
                            (long)target_cdb, (unsigned)atk_ms, (unsigned)dcy_ms,
                            (unsigned)hang_ms);
            else
                bsp_uprintf("I2C cmd: AGC OFF (fixed rx_audio_gain)\r\n");
        } else {
            bsp_uprintf("I2C cmd: AGC bad len %u (need >= 7)\r\n", (unsigned)n);
        }
        break;
    case CMD_LO_OFFSET:
        rx_lo_offset_hz = (int32_t)arg;           /* signed Hz, two's-complement */
        bsp_uprintf("I2C cmd: LO_OFFSET %ld Hz (ftuned = %lu + offset)\r\n",
                    (long)rx_lo_offset_hz, (unsigned long)rx_cur_station_hz);
        apply_tune(rx_cur_station_hz);            /* re-apply LO at the current station */
        break;
    case CMD_MODE: {
        uint8_t m = (uint8_t)(arg & 0xFFu);
        if (m < RX_MODE_AM || m > RX_MODE_USB) {
            bsp_uprintf("I2C cmd: MODE bad value %u (1=AM 2=LSB 3=USB)\r\n", (unsigned)m);
            break;
        }
        rx_mode = (int)m;
        /* SSB wants the LO on the dial (offset 0) so the dial offset maps to
         * audio pitch; AM uses its default offset. Override later with LO_OFFSET. */
        rx_lo_offset_hz = (m == RX_MODE_AM) ? AM_LO_OFFSET : 0;
        bsp_uprintf("I2C cmd: MODE %s (lo_offset -> %ld Hz)\r\n",
                    m == RX_MODE_AM ? "AM" : m == RX_MODE_LSB ? "LSB" : "USB",
                    (long)rx_lo_offset_hz);
        apply_tune(rx_cur_station_hz);            /* re-apply LO + flush the glitch */
        break;
    }
    case CMD_RFPRIM:
        bsp_uprintf("I2C cmd: RFPRIM idx=%lu\r\n", (unsigned long)arg);
        rf_apply_primary((int)(arg & 0xFu));
        break;
    case CMD_RFSEC:
        if (arg <= 6u) {
            int sbit = 13 - (int)arg;
            rf_image = (rf_image & ~0x3F80u) | (1u << sbit);
            rf_commit_image(rf_image);
            bsp_uprintf("I2C cmd: RFSEC idx=%lu -> image bit %d, img=0x%06lX\r\n",
                        (unsigned long)arg, sbit, (unsigned long)rf_image);
        } else {
            bsp_uprintf("I2C cmd: RFSEC idx=%lu out of range (0..6)\r\n", (unsigned long)arg);
        }
        break;
    case CMD_BANDV:
        DAC->DHR12R2 = (uint16_t)(arg & 0xFFFu);
        bsp_uprintf("I2C cmd: BANDV DAC2=0x%03lX\r\n", (unsigned long)(arg & 0xFFFu));
        break;
    case CMD_RFIMG:
        rf_image = arg & 0xFFFFFFu;
        rf_commit_image(rf_image);
        bsp_uprintf("I2C cmd: RFIMG img=0x%06lX (SPI6/74HC595)\r\n", (unsigned long)rf_image);
        break;
    case CMD_SC6:
        rf_chain6((uint8_t)(arg & 0x3Fu));
        bsp_uprintf("I2C cmd: SC6 word=0x%02lX (PB1-latched RF chain)\r\n",
                    (unsigned long)(arg & 0x3Fu));
        break;
    case CMD_SC8PG6:
        rf_chain_pg6((uint8_t)(arg & 0xFFu));
        bsp_uprintf("I2C cmd: SC8PG6 word=0x%02lX (PG6-latched RF chain)\r\n",
                    (unsigned long)(arg & 0xFFu));
        break;
    case CMD_GPIO: {
        static GPIO_TypeDef * const ports[7] =
            { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG };
        unsigned port = arg & 0xFu, pin = (arg >> 4) & 0xFu, mode = (arg >> 8) & 0xFu;
        if (port > 6u || pin > 15u) {
            bsp_uprintf("I2C cmd: GPIO bad port/pin (p=%u pin=%u)\r\n", port, pin);
            break;
        }
        GPIO_TypeDef *g = ports[port];
        /* Set the output latch BEFORE switching MODER to output (no glitch). */
        switch (mode) {
        case 0: M_IN(g, pin); break;
        case 1: M_IN(g, pin); PULL_DN(g, pin); break;
        case 2: M_IN(g, pin); PULL_UP(g, pin); break;
        case 3: SET_LO(g, pin); M_PP(g, pin); M_OUT(g, pin); break;
        case 4: SET_HI(g, pin); M_PP(g, pin); M_OUT(g, pin); break;
        case 5: SET_LO(g, pin); M_OD(g, pin); M_OUT(g, pin); break;
        case 6: SET_HI(g, pin); M_OD(g, pin); M_OUT(g, pin); break;
        default: bsp_uprintf("I2C cmd: GPIO bad mode %u\r\n", mode); goto gpio_done;
        }
        bsp_uprintf("I2C cmd: GPIO P%c%u mode=%u (IDR=%u)\r\n",
                    (char)('A' + port), pin, mode,
                    (unsigned)((g->IDR >> pin) & 1u));
    gpio_done:
        break;
    }
    case CMD_CODEC: {
        uint8_t page = (uint8_t)(arg & 0xFFu);
        uint8_t reg  = (uint8_t)((arg >> 8) & 0xFFu);
        uint8_t val  = (uint8_t)((arg >> 16) & 0xFFu);
        uint8_t sel[2] = {0x00u, page};
        uint8_t wr[2]  = {reg, val};
        int rc = cwr(ADDR_CODEC, sel, 2);
        rc |= cwr(ADDR_CODEC, wr, 2);
        bsp_uprintf("I2C cmd: CODEC P%u R0x%02X = 0x%02X%s\r\n",
                    page, reg, val, rc ? " (NAK!)" : "");
        break;
    }
    case CMD_DUMP:
        bsp_uprintf("I2C cmd: DUMP\r\n");
        rf_dump();
        break;
    default:
        bsp_uprintf("I2C cmd: unknown op 0x%02X (len=%u)\r\n", op, (unsigned)n);
        break;
    }
}

/* ======================================================================== *
 *  Codec / converter startup (i2c.md; CSV sequences). Codec at 0x18.
 * ======================================================================== */
static int cwr(uint8_t addr, const uint8_t *buf, uint32_t n)
{
    return i2c_write_raw((addr == ADDR_MAX17205) ? I2C1 : I2C2, addr, buf, n);
}

/* TLV320AIC3204 100 kHz clock setup (FUN_0802f0ec) -- the exact register
 * blocks the firmware emits for a 100000-Hz converter rate. */
static void tlv320_clock_100k(void)
{
    static const uint8_t page0[]   = {0x00, 0x00};
    static const uint8_t pll[]     = {0x04, 0x40, 0xC1, 0x0F, 0x1D, 0x08, 0x74};
    static const uint8_t dosr[]    = {0x0D, 0x00, 0x80}; /* DOSR = 128        */
    static const uint8_t aosr[]    = {0x14, 0x80};       /* AOSR = 128        */
    static const uint8_t ndac_mdac[] = {0x0B, 0x82, 0x84}; /* NDAC=2,MDAC=4 pw */
    static const uint8_t nadc_madc[] = {0x12, 0x02, 0x04}; /* NADC=2,MADC=4    */
    cwr(ADDR_CODEC, page0, sizeof page0);
    cwr(ADDR_CODEC, pll, sizeof pll);
    cwr(ADDR_CODEC, dosr, sizeof dosr);
    cwr(ADDR_CODEC, aosr, sizeof aosr);
    cwr(ADDR_CODEC, ndac_mdac, sizeof ndac_mdac);
    cwr(ADDR_CODEC, nadc_madc, sizeof nadc_madc);
}

/* The 58-step startup (tlv320aic3204_startup_sequence.csv). Step 3 is the clock
 * helper above; the 10-ms delay is CSV step 9. Each pair is {register,value};
 * {0x00,page} selects a page. */
static const uint8_t tlv_pre[][2] = {   /* CSV steps 1,2 then 4-8 */
    {0x04, 0x43}, {0x1D, 0x00}, {0x1E, 0x88}, {0x1B, 0x3C},
};
static const uint8_t tlv_post[][2] = {  /* CSV steps 10-58 */
    {0x00, 0x00}, {0x19, 0x06}, {0x1A, 0x82}, {0x37, 0x06}, {0x3C, 0x19}, {0x3D, 0x01},
    {0x00, 0x01}, {0x01, 0x08}, {0x0A, 0x03}, {0x3D, 0x00}, {0x03, 0x00}, {0x04, 0x00},
    {0x47, 0x32}, {0x7B, 0x05},
    {0x00, 0x00}, {0x51, 0xC0}, {0x52, 0x00}, {0x53, 0x00}, {0x54, 0x00},
    {0x00, 0x01}, {0x09, 0x3F}, {0x0C, 0x08}, {0x0D, 0x08}, {0x0E, 0x08}, {0x0F, 0x08},
    {0x10, 0x40}, {0x11, 0x40}, {0x12, 0x40}, {0x13, 0x40}, {0x14, 0x25},
    {0x00, 0x00}, {0x3F, 0xD6}, {0x40, 0x00}, {0x41, 0x00}, {0x42, 0x00},
    {0x00, 0x00}, {0x53, 0x6C}, {0x00, 0x00}, {0x54, 0x00}, {0x00, 0x00}, {0x41, 0x00},
    {0x00, 0x00}, {0x42, 0x00},
    {0x00, 0x01}, {0x34, 0x10}, {0x36, 0x40}, {0x00, 0x01}, {0x37, 0x14}, {0x39, 0x80},
};

static void tlv320_startup(void)
{
    static const uint8_t page0[] = {0x00, 0x00};
    static const uint8_t reset[] = {0x01, 0x01};
    cwr(ADDR_CODEC, page0, sizeof page0);        /* step 1 */
    cwr(ADDR_CODEC, reset, sizeof reset);        /* step 2 */
    bsp_delay_ms(1);

    /* AVDD LDO ramp BEFORE the PLL -- the piece that made the working experiment
     * firmwares/phase2b-sample-transport-250 lock. Page-1 0x02=0x01 powers the
     * codec's internal analog LDO; without it the analog PLL has no supply and
     * never locks (R0x05 shows PLL "ON" but no BCLK/WCLK is produced). */
    {
        static const uint8_t ldo_p1[] = {0x00, 0x01};   /* page 1              */
        static const uint8_t ldo_on[] = {0x02, 0x01};   /* AVDD LDO powered up */
        static const uint8_t ldo_p0[] = {0x00, 0x00};   /* back to page 0      */
        cwr(ADDR_CODEC, ldo_p1, sizeof ldo_p1);
        cwr(ADDR_CODEC, ldo_on, sizeof ldo_on);
        cwr(ADDR_CODEC, ldo_p0, sizeof ldo_p0);
        bsp_delay_ms(10);
    }

    tlv320_clock_100k();                         /* step 3 */
    for (unsigned i = 0; i < sizeof tlv_pre / 2; i++)
        cwr(ADDR_CODEC, tlv_pre[i], 2);          /* steps 4-8 */
    bsp_delay_ms(10);                            /* step 9 */
    for (unsigned i = 0; i < sizeof tlv_post / 2; i++)
        cwr(ADDR_CODEC, tlv_post[i], 2);         /* steps 10-58 */
}

/* Unmute the headphone drivers. The OEM startup table (tlv_post, steps 35-36)
 * brings HPL/HPR up MUTED -- page 1 R0x10/R0x11 = 0x40 = mute bit set, 0-dB gain
 * -- and the OEM only unmutes HPR later, at runtime, from its rxvol
 * (receive-volume) control (FUN_0802c7d0). phase3-rx replicates the startup
 * table but has no rxvol path, so the DAC plays into a muted HPR driver = dead
 * silence on the speaker even at full digital VOLUME. Clear the mute bit (0x00 =
 * unmuted, 0 dB) on both drivers; digital rx_audio_gain still sets loudness and
 * HPR feeds the LM386 -> internal speaker (gpio.md PG1 enable). Tunable live via
 * CMD_CODEC if 0 dB is too quiet into the LM386. */
static void codec_unmute_hp(void)
{
    /* Unmute at +9 dB (0x09 = bit6 clear, signed-6-bit gain = +9). With the AGC
     * on by default the digital path carries the receive gain (the AGC normalizes
     * the weak I/Q up to target), so the HP driver only needs modest analog makeup
     * -- +9 dB puts a comfortable loudness near VOLUME ~1.0 (unity). With the AGC
     * off (fixed-gain fallback) a much larger VOLUME is needed. Tune live over
     * I2C with CMD_CODEC ('Z 1 0x10/0x11 <gain>'): 0x00=0 dB .. 0x1D=+29 dB. */
    static const uint8_t p1[]  = {0x00, 0x01};   /* select page 1                */
    static const uint8_t hpl[] = {0x10, 0x09};   /* HPL driver: unmute, +9 dB    */
    static const uint8_t hpr[] = {0x11, 0x09};   /* HPR driver: unmute, +9 dB    */
    cwr(ADDR_CODEC, p1, sizeof p1);
    cwr(ADDR_CODEC, hpl, sizeof hpl);
    cwr(ADDR_CODEC, hpr, sizeof hpr);
}

/* 0x4A six-write companion (FUN_08030754). */
static void aux4a_startup(void)
{
    static const uint8_t w1[] = {0x00, 0x00};
    static const uint8_t w2[] = {0x20, 0x01};
    static const uint8_t w3[] = {0x00, 0x00, 0x80, 0x3F, 0x00};
    static const uint8_t w4[] = {0x0B, 0x00};
    static const uint8_t w5[] = {0x0D, 0x01};
    static const uint8_t w6[] = {0x0E, 0x01};
    cwr(ADDR_AUX4A, w1, sizeof w1);
    cwr(ADDR_AUX4A, w2, sizeof w2);
    cwr(ADDR_AUX4A, w3, sizeof w3);
    cwr(ADDR_AUX4A, w4, sizeof w4);
    bsp_delay_ms(1);
    cwr(ADDR_AUX4A, w5, sizeof w5);
    bsp_delay_ms(1);
    cwr(ADDR_AUX4A, w6, sizeof w6);
}

/* CS4272 probe + init (FUN_0803022c), or 0x4C fallback (FUN_080244b4).
 * Returns 1 = CS4272 configured, 0 = 0x4C fallback used. */
static int iq_converter_startup(void)
{
    static const uint8_t probe[] = {0x07, 0x03};  /* Mode Control 2, powered down */
    if (i2c_write_raw(I2C2, ADDR_CS4272, probe, sizeof probe) == 0) {
        static const uint8_t seq[][2] = {
            {0x01, 0x81}, {0x02, 0x00}, {0x03, 0x49}, {0x04, 0x00},
            {0x05, 0x00}, {0x06, 0x10}, {0x07, 0x02},
        };
        for (unsigned i = 0; i < sizeof seq / 2; i++)
            cwr(ADDR_CS4272, seq[i], 2);
        return 1;
    }
    /* fallback at 0x4C */
    bsp_delay_ms(10);
    static const uint8_t f1[] = {0x10, 0x00};
    static const uint8_t f2[] = {0x10, 0xC2};
    static const uint8_t f3[] = {0x11, 0x80};
    static const uint8_t f4[] = {0x14, 0x00};
    cwr(ADDR_AUX4C, f1, sizeof f1);
    bsp_delay_ms(10);
    cwr(ADDR_AUX4C, f2, sizeof f2);
    cwr(ADDR_AUX4C, f3, sizeof f3);
    cwr(ADDR_AUX4C, f4, sizeof f4);
    return 0;
}

static void codec_reset_pulse(void)
{
    /* PE15 = TLV320 active-low hardware reset (gpio.md/i2c.md): low >=1 ms,
     * then high before the codec register sequence. */
    SET_LO(GPIOE, 15);
    bsp_delay_ms(1);
    SET_HI(GPIOE, 15);
    bsp_delay_ms(1);
}

#ifdef PHASE2_DIAG
/* No-scope check: is the codec actually producing WCLK/BCLK? The codec's clock
 * outputs are wired to the STM32 I2S pins, so sample them as plain GPIO inputs
 * BEFORE i2s_config() muxes them to AF5. WS=PB12 (~100 kHz WCLK), CK=PB13
 * (~6.4 MHz BCLK). A pin that shows both 0 and 1 across the burst is toggling. */
static void i2s_clock_sniff(void)
{
    GPIOB->MODER &= ~((3u << (12u * 2)) | (3u << (13u * 2)));  /* PB12/PB13 = input */
    uint32_t ws_lo = 0, ws_hi = 0, ck_lo = 0, ck_hi = 0;
    for (uint32_t i = 0; i < 2000000u; i++) {
        uint32_t idr = GPIOB->IDR;
        if (idr & (1u << 12)) ws_hi++; else ws_lo++;
        if (idr & (1u << 13)) ck_hi++; else ck_lo++;
    }
    bsp_uprintf("  I2S clock sniff (pre-AF): WS/PB12 lo=%lu hi=%lu  CK/PB13 lo=%lu hi=%lu\r\n",
                (unsigned long)ws_lo, (unsigned long)ws_hi,
                (unsigned long)ck_lo, (unsigned long)ck_hi);
    bsp_uprintf("  -> %s\r\n",
        ((ws_lo && ws_hi) || (ck_lo && ck_hi))
          ? "CLOCK PRESENT: codec IS mastering -> STM32 I2S config bug"
          : "NO CLOCK: codec not emitting BCLK/WCLK -> PLL/MCLK/OUT0 upstream");
}
#endif /* PHASE2_DIAG */

int main(void)
{
    __enable_irq();   /* the IAP bootloader may hand off with IRQs masked; this
                       * firmware's only interrupt is the DMA1_Stream3 TC that
                       * drives bank_count (SysTick here is polled, not IRQ). */
    bsp_gpio_init_safe();
    bsp_clock_init_250mhz();
    bsp_systick_init_1ms();
    bsp_usart1_init();

    /* DWT cycle counter: the per-bank DSP CPU-cost gauge (rx_dsp_cyc_max). Must
     * be live before the I2S DSP ISR runs. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    bsp_uprintf("\r\n=== %s ===\r\n", FW_VERSION);
    bsp_uprintf("clock: HCLK=%lu Hz (%s); PCLK1=%lu PCLK2=%lu\r\n",
                (unsigned long)HCLK_HZ, BSP_CLK_DESC,
                (unsigned long)PCLK1_HZ, (unsigned long)PCLK2_HZ);
    bsp_uprintf("safe-state held: PA-tuple disabled, charger off, +5T off\r\n");

    i2c_pins_init();
    i2c_master_init(I2C1);
    i2c_master_init(I2C2);

    /* ---- ADC + DAC ---- */
    adc_init();
    dac_init();
    DAC->DHR12R2 = 0x6B0u;   /* example BAND_V band code (frequency_control.md) */
    bsp_uprintf("-- analog --\r\n");
    bsp_uprintf("  ADC1 ch0/1/2 + ADC2 ch6/7/14 scan+DMA2 configured; DAC init, DAC2 code=0x%03lX\r\n",
                (unsigned long)(DAC->DHR12R2 & 0xFFFu));

    /* External-voltage scaling self-test (filter math is model-independent). */
    {
        const uint16_t raw = 2482u;    /* example PA6 reading */
        uint16_t tbuf[15];
        for (int i = 0; i < 15; i++) tbuf[i] = (i % 3 == 0) ? raw : 0u;
        float save = adc2_y[0]; adc2_y[0] = adc2_y[1] = adc2_y[2] = 0.0f;
        for (int i = 0; i < 400; i++) adc2_filter_step(tbuf);
        float mv = external_voltage_mv();
        bsp_uprintf("  ext-voltage self-test: raw=%u getter0=%u -> %lu mV (%lu.%01lu V)\r\n",
                    raw, (unsigned)adc2_y[0], (unsigned long)mv,
                    (unsigned long)(mv / 1000.0f), ((unsigned long)(mv / 100.0f)) % 10u);
        adc2_y[0] = adc2_y[1] = adc2_y[2] = save;
    }
    /* Live ADC path (hardware-driven; zero in the emulator with no analog). */
    adc2_filter_step((const uint16_t *)adc2_buf);
    bsp_uprintf("  live ADC2 getter0=%u (0 without external analog / live ADC)\r\n",
                (unsigned)adc2_y[0]);

    /* ---- MAX17205 telemetry ---- */
    bsp_uprintf("-- MAX17205 telemetry --\r\n");
    uint8_t blk[0x1B6];
    if (i2c_read(I2C1, ADDR_MAX17205, 0x00u, blk, sizeof blk) == 0) {
        uint16_t repsoc = le16(&blk[0x0C]);
        int16_t  temp   = (int16_t)le16(&blk[0x10]);
        int16_t  avgcur = (int16_t)le16(&blk[0x16]);
        uint16_t batt   = le16(&blk[0x1B4]);
        bsp_uprintf("  RepSOC=%lu%%  Batt=%lu mV  Temp=%ld C  AvgCurrent=%ld (x0.15625)\r\n",
                    (unsigned long)(repsoc * 39UL / 10000UL),       /* x0.0039 */
                    (unsigned long)(batt * 125UL / 100UL),          /* x1.25 mV */
                    (long)(temp / 256),                             /* signed /256 */
                    (long)((avgcur * 15625L) / 100000L));           /* x0.15625 */
    } else {
        bsp_uprintf("  MAX17205 read FAILED\r\n");
    }

    /* ---- VersaClock sample clock (codec MCLK source) ---- */
    bsp_uprintf("-- VersaClock (I2C1 0x6A) --\r\n");
    int vc = versaclock_init();
    bsp_uprintf("  0x6A %s; OUT0=26 MHz -> TLV320 MCLK (codec PLL ref)\r\n",
                vc ? "ACK (present)" : "NAK (absent -- check PE14/PE7/rail)");

    /* ---- Codec + converters ---- */
    bsp_uprintf("-- audio codec / IQ converter --\r\n");
    codec_reset_pulse();
    tlv320_startup();
    bsp_delay_ms(100);               /* i2c.md step 5 */
    codec_unmute_hp();               /* OEM leaves HPL/HPR muted; we want sound */
    aux4a_startup();
    SET_HI(GPIOG, 8);                /* set PG8 ODR (latch; PG8 stays input)  */
    int cs = iq_converter_startup();
    bsp_uprintf("  TLV320 init done (3+58 steps); 0x4A companion done; IQ=%s\r\n",
                cs ? "CS4272" : "0x4C fallback");

#ifdef PHASE2_DIAG
    /* Clock-chain read-back. The 58-step startup leaves the codec on PAGE 1, so
     * select page 0 first -- the earlier bare R0x1B/R0x09 reads hit page-1
     * registers (R0x1B=0x00 was a red herring). Dumping the page-0 PLL /
     * interface / BCLK registers tells us whether the codec's master-clock
     * config actually stuck: if PLL-power (R0x05 bit7) + master iface (R0x1B=0x3C)
     * read back correct but banks stay 0, MCLK isn't really arriving (VersaClock
     * OUT0) or the PLL isn't locking -> scope MCLK/BCLK next. */
    {
        static const uint8_t page0[] = {0x00, 0x00};
        cwr(ADDR_CODEC, page0, sizeof page0);
        static const struct { uint8_t reg; const char *what; } cregs[] = {
            {0x04, "CLKIN mux   (exp 0x43: CODEC_CLKIN=PLL, PLL_CLKIN=MCLK)"},
            {0x05, "PLL P/R+pwr (exp 0xC1: PLL ON, P=4, R=1)"},
            {0x06, "PLL J       (exp 0x0F=15)"},
            {0x07, "PLL D msb   (exp 0x1D)"},
            {0x08, "PLL D lsb   (exp 0x08)"},
            {0x0B, "NDAC        (exp 0x82: ON, N=2)"},
            {0x0C, "MDAC        (exp 0x84: ON, M=4)"},
            {0x1B, "iface       (exp 0x3C: 32-bit I2S, BCLK/WCLK OUT = master)"},
            {0x1D, "iface2      (exp 0x00)"},
            {0x1E, "BCLK N div  (exp 0x88: ON, N=8)"},
        };
        bsp_uprintf("  codec page-0 clock chain:\r\n");
        for (unsigned i = 0; i < sizeof cregs / sizeof cregs[0]; i++) {
            uint8_t v = 0;
            if (i2c_read(I2C2, ADDR_CODEC, cregs[i].reg, &v, 1) == 0)
                bsp_uprintf("    R0x%02X=0x%02X  %s\r\n", cregs[i].reg, v, cregs[i].what);
            else
                bsp_uprintf("    R0x%02X=NAK\r\n", cregs[i].reg);
        }
    }
#endif /* PHASE2_DIAG */

    bsp_uprintf("codec bring-up complete (TLV320 is I2S clock master)\r\n");

    /* ---- RX front-end: LO + RF band switching + Linux control slave ---- */
    bsp_uprintf("-- RX front-end --\r\n");
    spi6_init();

    /* PD15 = constant RX-path enable. The OEM drives it high in every RX dump
     * (both bands); common/bsp.c leaves it in its reset state. Make it a
     * push-pull output and assert it high before tuning. */
    M_OUT(GPIOD, 15); M_PP(GPIOD, 15); SPD_HI(GPIOD, 15); SET_HI(GPIOD, 15);
    bsp_uprintf("  PD15 RX-path enable = HIGH\r\n");

    /* Mirror the OEM startup serial-chain write exactly: the stock FW's init
     * (FUN_0802a5d0) issues a single cf88(0x0E) on the PB1-latched 6-bit chain
     * and NEVER touches the PG6 chain (oem-rftrace, 2026-11-02). apply_tune()
     * re-drives cf88 with the band word (0x25 for <=2 MHz) below, so this just
     * reproduces the OEM's exact power-on-then-tune sequence. */
    rf_chain6(0x0Eu);
    bsp_uprintf("  RF startup chain6 (PB1 latch) = 0x0E (OEM default)\r\n");

    i2c3_slave_init();
    bsp_uprintf("  I2C3 Linux control slave @ 0x72 (PA8/PC9): TUNE=0x01 u32 Hz, "
                "VOLUME=0x02 f32, AGC=0x03 i16+u16+u16[+u16], LO_OFFSET=0x04 i32 Hz, "
                "MODE=0x05 u8(1=AM 2=LSB 3=USB), LINMULT=0x06 f32\r\n");
    bsp_uprintf("  LO offset default = %ld Hz; mode = AM\r\n", (long)rx_lo_offset_hz);
    bsp_uprintf("  initial TUNE=%lu Hz, VOLUME=0.0 (silent until set from R2); "
                "LO OUT1 = 4 x Ftuned\r\n", (unsigned long)RX_TUNED_HZ);
    apply_tune(RX_TUNED_HZ);

    /* AGC ON by default (VOLUME still 0 = silent until set). */
    rx_agc_set(RX_AGC_DEF_TARGET_CDB, RX_AGC_DEF_ATK_MS, RX_AGC_DEF_DCY_MS,
               RX_AGC_DEF_HANG_MS);
    bsp_uprintf("  AGC ON (default): target=%d cdBFS atk=%u dcy=%u hang=%u ms\r\n",
                RX_AGC_DEF_TARGET_CDB, RX_AGC_DEF_ATK_MS, RX_AGC_DEF_DCY_MS,
                RX_AGC_DEF_HANG_MS);

    /* ---- I2S sample transport + AM receive DSP (Phase 3 rx_dsp.c) ---- */
    bsp_uprintf("-- I2S sample transport + AM DSP --\r\n");
    bsp_uprintf("  DSP rates: raw=%lu primary=%lu audio=%lu alt=%lu Hz; audio gain=%ld/1000\r\n",
                (unsigned long)i2s_rate_raw, (unsigned long)i2s_rate_primary,
                (unsigned long)i2s_rate_audio, (unsigned long)i2s_rate_alt,
                (long)(rx_audio_gain * 1000.0f));
#ifdef PHASE2_DIAG
    i2s_clock_sniff();   /* no-scope: does the codec actually clock WCLK/BCLK? */
#endif
    i2s_config();
    i2s_start();
    bsp_uprintf("  4 DMA1 streams live; DMA1_Stream3 ISR runs the AM demod -> "
                "I2S2ext audio out\r\n");
    bsp_uprintf("  I/Q TX pair stays zero (inactive); PA off, charger off, +5T off\r\n");
    bsp_uprintf("phase3-rx running: listen on headphone/speaker for WDOV\r\n");

    uint32_t ms = 0, last_banks = 0;
    /* Cycles available per 1.28 ms bank (128 I/Q samples @ 100 kHz) for the CPU%
     * gauge. The DSP runs in the DMA ISR, so this is its worst-case bank budget. */
    const uint32_t budget_cyc = (HCLK_HZ / 100000u) * I2S_BANK_FRAMES;
    for (;;) {
        bsp_delay_ms(1);
        i2c3_poll_commands();   /* apply any TUNE/VOLUME from the R2 */
        if ((++ms % 1000u) == 0u) {
            uint32_t banks = i2s_bank_count;
            uint32_t cyc_max = rx_dsp_cyc_max; rx_dsp_cyc_max = 0;  /* peak this period, then clear */
            uint32_t cpu_pct = budget_cyc ? (cyc_max * 100u / budget_cyc) : 0u;
            /* DSP levels as integer micro-units (no float printf under nano). */
            bsp_uprintf("alive t=%lu ms  %s  banks=%lu (+%lu/s)  iq_rms=%ld env_dc=%ld "
                        "audio_pk=%ld (x1e-6)  agc=%s%ld dB  cpu_max=%lu%% (%lu cyc)\r\n",
                        (unsigned long)ms,
                        rx_mode == RX_MODE_AM ? "AM" : rx_mode == RX_MODE_LSB ? "LSB" : "USB",
                        (unsigned long)banks,
                        (unsigned long)(banks - last_banks),
                        (long)(rx_iq_rms * 1000000.0f),
                        (long)(rx_env_dc * 1000000.0f),
                        (long)(rx_audio_peak * 1000000.0f),
                        rx_agc_on ? "" : "off ",
                        rx_agc_on ? (long)(rx_agc_gain_db + (rx_agc_gain_db < 0.0f ? -0.5f : 0.5f)) : 0L,
                        (unsigned long)cpu_pct, (unsigned long)cyc_max);
#ifdef PHASE2_DIAG
            /* Is any BCLK/WCLK reaching the STM32? SR life (RXNE b0 / OVR b6 /
             * BSY b7 / FRE b8) or NDTR moving off 512 = clock present -> DMA-side
             * bug; SR idle + NDTR stuck = no clock -> break is upstream (OUT0/codec). */
            bsp_uprintf("  i2s SR SPI2=0x%04lX SPI3=0x%04lX  NDTR S0=%lu S3=%lu  "
                        "isr=%lu LISR=0x%08lX\r\n",
                        (unsigned long)(SPI2->SR & 0xFFFFu), (unsigned long)(SPI3->SR & 0xFFFFu),
                        (unsigned long)DMA1_Stream0->NDTR, (unsigned long)DMA1_Stream3->NDTR,
                        (unsigned long)i2s_isr_entries, (unsigned long)DMA1->LISR);
#endif
            last_banks = banks;
        }
    }
}
