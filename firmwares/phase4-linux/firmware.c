/*
 * x6100-bbfw -- Phase 4: OEM Linux control plane over the Phase 3 receiver
 * =======================================================================
 * This is the Phase 3 receive-only radio (described below) with its custom
 * 5-byte I2C command slave REPLACED by the OEM BASE 1.1.9 Linux hardware
 * contract (docs/protocol.md + docs/linux_hardware_interface.md):
 *   - I2C3 @ 0x72 OEM register window: 56-word control file + fw-id read + reg55
 *     boot handshake + calib/cmd windows;
 *   - USART6 /dev/ttyS1 0x1028-byte IQ+telemetry flow packets (DMA2 Stream 6,
 *     hardware CRC32) at 1.5 Mbaud, ~35 ms cadence;
 *   - control_scan_apply(): drives the RX engine from the OEM words -- reg1/7
 *     VFO freq, reg4/10 mode, reg13 active VFO, reg14 rxvol, reg5/11 AGC mode +
 *     reg27/28 AGC hang/time, reg15 RF gain -> apply_tune() / rx_mode /
 *     rx_audio_gain / rx_agc_set / rx_rf_gain.
 * The reg55 handshake is TRACKED, not gated, so standalone RX (no Linux app) is
 * preserved. Flow IQ is zero-filled in this first merged build (RX audio is on
 * the BASE codec, already proven); live IQ + telemetry are the next increment.
 * Diagnostics on USART1 -> /dev/ttyACM1 @115200 (not part of the Linux contract).
 *
 * ---- Phase 3 receive-only radio (the RX engine below is unchanged) ----
 * Receive-only radio (AM on 1410 kHz, WDOV).
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
#include <math.h>    /* powf for the OEM rxvol -> digital-scalar transfer (libm) */

#include "bsp.h"
#include "i2c.h"
#include "i2s.h"
#include "rx_dsp.h"

#define FW_VERSION "x6100-bbfw phase4-linux 2026-11-11"

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
/* AGC release hang when the OEM reg27 agchang bit (bit 12) is set: the AGC waits
 * before letting gain rise again (holds through a short pause between words). */
#define RX_AGC_HANG_MS         250u

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

/* MAX17205 registers (page 0, I2C 0x36). */
#define MAX17205_REG_AVGCURRENT 0x0Bu   /* signed, x0.15625 mA / Rsense         */
#define MAX17205_REG_BATT       0xDAu   /* cell-stack voltage, x1.25 mV         */
#define BAT_CHG_RAW_MIN         320     /* AvgCurrent raw for ~+50 mA into pack  */

/* Latest battery pack voltage (mV) + charge current, refreshed ~1 Hz from the
 * main loop. Seeded at boot from the startup MAX17205 read. flow_build() turns
 * these into the flow packet's vbat/batcap/flags. */
static volatile float   g_bat_mv;
static volatile int32_t g_bat_cur_raw;   /* AvgCurrent raw (signed) */

/* Battery percentage from a 2S Li-ion discharge curve (Georgy x6100_base_patch
 * utils.c bat_cap_by_voltage). Voltage-based, NOT the MAX17205 RepSOC gauge --
 * robust even on an imperfectly-provisioned gauge, and consistent with the
 * OpenX6100 BASE the GUI expects. */
static uint8_t bat_cap_by_voltage(float bat_mv)
{
    static const struct { uint16_t mv; uint8_t pct; } lut[] = {
        {8400, 100}, {8100, 94}, {7838, 77}, {7609, 62}, {7508, 53},
        {7414, 41}, {7326, 27}, {7243, 15}, {7166, 8}, {7020, 3}, {6000, 0},
    };
    const unsigned n = sizeof lut / sizeof lut[0];
    if (bat_mv >= (float)lut[0].mv)     return 100u;
    if (bat_mv <= (float)lut[n - 1].mv) return 0u;
    for (unsigned i = 0; i < n - 1u; i++) {
        if (bat_mv >= (float)lut[i + 1].mv && bat_mv < (float)lut[i].mv) {
            float t = (bat_mv - lut[i + 1].mv) / ((float)lut[i].mv - lut[i + 1].mv);
            return (uint8_t)(lut[i + 1].pct + t * (lut[i].pct - lut[i + 1].pct) + 0.5f);
        }
    }
    return 0u;
}

/* Refresh g_bat_mv / g_bat_cur_raw from the MAX17205 (blocking I2C1; call ~1 Hz
 * from the main loop -- not from the flow path). Leaves the last good value on a
 * bus error. */
static void bat_telemetry_poll(void)
{
    uint8_t b[2];
    if (i2c_read(I2C1, ADDR_MAX17205, MAX17205_REG_BATT, b, 2) == 0)
        g_bat_mv = (float)le16(b) * 1.25f;              /* x1.25 mV */
    if (i2c_read(I2C1, ADDR_MAX17205, MAX17205_REG_AVGCURRENT, b, 2) == 0)
        g_bat_cur_raw = (int32_t)(int16_t)le16(b);      /* signed */
}

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
static void rf_chain_pg6(uint8_t word) __attribute__((unused));
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
 *  Linux control plane: OEM-compatible I2C3 register window + boot handshake.
 *  ------------------------------------------------------------------------
 *  phase4 swaps phase3-rx's custom 5-byte command slave for the OEM BASE 1.1.9
 *  I2C3 register-file contract (docs/protocol.md), so the stock Linux app (or
 *  phase4-linux-ctl.py) drives the radio through the real 56-word control file.
 *  Same slave electrically: 7-bit 0x72 (OAR1 = 0x00E4), PA8 SCL / PC9 SDA AF4
 *  (Linux reaches it via /dev/i2c-0). Framing: a write is [addr-hi][addr-lo]
 *  [payload LE...], byte address = reg_index*4; a read is an offset-only write,
 *  STOP, then a separate read -- so the slave KEEPS the offset across the STOP.
 *  The 16-bit address selects a different RAM window by transfer DIRECTION
 *  (control file / calib / cmd on write; fw-id / calib / 0x61 on read). The
 *  56-word control file is scanned in the main loop and mapped onto the RX
 *  engine (control_scan_apply). Register 55 = Linux boot-ready handshake.
 * ======================================================================== */
#define I2C3_ADDR8   0xE4u     /* 7-bit 0x72 in the OAR1 shifted form */
/* ---- Register-55 Linux boot handshake (docs/protocol.md:649) ----
 * Byte addr 0x00DC, control word index 55, ready bit 0x00100000. In phase4 the
 * handshake is TRACKED (logged), not gated: we keep phase3-rx's standalone RX,
 * so the radio receives with or without the Linux app; reg55 only reports when
 * X6100Control has come up. */
#define REG_HANDSHAKE_INDEX   55u
#define HANDSHAKE_READY_BIT   0x00100000u

/* ---- OEM register-file RAM windows (sizes/offsets per docs/protocol.md) ----
 * Control file: 56 u32 = 0xE0 bytes. Linux writes here; it is NOT what Linux
 * reads back at offset 0 (that is the fw-id). Each window is circular. */
static volatile union {
    uint8_t  b[0xE0];
    uint32_t w[56];
} ctrl;
static uint8_t  calib[0x330];           /* calibration window (shared R/W)      */
static uint8_t  cmdwin[0x100];          /* 0xFFFE-0xFFFF command window          */
static uint8_t  fwid[0x80];             /* 0x0000-0x007F firmware-id read win    */

/* I2C3 slave transaction state (ISR + main). */
static volatile uint16_t i2c_off;       /* selected byte offset, PERSISTS across STOP */
static volatile uint8_t  i2c_phase;     /* write framing: 0=addr-hi 1=addr-lo 2=payload */
static volatile uint32_t i2c_isr_calls; /* EV ISR entries (diagnostic)          */
static volatile uint32_t i2c_rx_bytes;  /* bytes clocked in from the master      */
static volatile uint32_t i2c_stops;     /* STOP conditions seen                  */

/* Direction-dependent window access (docs/protocol.md). */
static void win_write(uint16_t off, uint8_t v)
{
    if (off >= 0xFFFEu)      cmdwin[(uint16_t)(off - 0xFFFEu) & 0xFFu] = v;
    else if (off >= 0xF000u) calib[(off - 0xF000u) % 0x330u] = v;
    else                     ctrl.b[off % 0xE0u] = v;        /* normal control file */
}
static uint8_t win_read(uint16_t off)
{
    if (off <= 0x007Fu)      return fwid[off & 0x7Fu];
    if (off <= 0x00FFu)      return 0x00u;
    if (off <= 0x0FFFu)      return calib[(off - 0x0100u) % 0x330u];
    if (off <= 0xEFFFu)      return 0x00u;                   /* undefined -> 0 */
    if (off <= 0xFFFDu)      return calib[(off - 0xF000u) % 0x330u];
    return 0x61u;                                            /* 0xFFFE-0xFFFF */
}

/* 128-byte firmware-id buffer: "V1.1.9" + build stamp (docs/protocol.md:693).
 * The V1.1.9 tag is what the stock Linux app checks to accept the pairing. */
static void fwid_init(void)
{
    for (unsigned i = 0; i < sizeof fwid; i++) fwid[i] = 0;
    static const char tag[] = "V1.1.9";
    /* ",r8" = the "patched BASE revision" the OpenX6100 GUI / X6100Control parse
     * (extract_version_info: strstr(",r") -> atoi). rev>=8 unlocks the enhanced
     * flow path: the GUI reads flow_info (lo_freq for absolute panadapter
     * positioning, fft_dec, flow_fmt) AND sets fw_dc_blocker=true (it trusts the
     * BASE to DC-block the IQ -- which we do, rx_dsp step 1b). rev 8 is the
     * minimal level that enables exactly this; it does NOT enable the rev>=9/12
     * TX-filter/CESSB paths we don't implement. Consistent with Georgy's
     * x6100_base_patch flow_info (comm.c). */
    static const char stamp[] = __DATE__ "," __TIME__ ",r8";
    unsigned n = 0;
    for (const char *p = tag;   *p && n < sizeof fwid; p++) fwid[n++] = (uint8_t)*p;
    for (const char *p = stamp; *p && n < sizeof fwid; p++) fwid[n++] = (uint8_t)*p;
}

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

/* I2C3 slave event handler (overrides the weak vector in common/startup.c).
 * Processes the OEM framing byte-by-byte and serves reads from the selected
 * window at the persistent offset. */
void I2C3_EV_IRQHandler(void)
{
    uint32_t sr1 = I2C3->SR1;
    i2c_isr_calls++;

    if (sr1 & I2C_SR1_ADDR) {                 /* address matched */
        (void)I2C3->SR1;                      /* explicit SR1 then SR2 read clears ADDR */
        uint32_t sr2 = I2C3->SR2;
        if (!(sr2 & I2C_SR2_TRA))             /* receiver: master is writing to us */
            i2c_phase = 0;                    /* next two bytes re-select the offset */
        /* transmitter (sr2 & TRA): serve from the persisted i2c_off on TXE */
    }

    if (sr1 & I2C_SR1_RXNE) {                 /* byte from master */
        uint8_t b = (uint8_t)I2C3->DR;
        i2c_rx_bytes++;
        if (i2c_phase == 0)      { i2c_off = (uint16_t)b << 8; i2c_phase = 1; }
        else if (i2c_phase == 1) { i2c_off |= b;              i2c_phase = 2; }
        else                     { win_write(i2c_off, b);     i2c_off++;     }
    }

    if ((sr1 & I2C_SR1_TXE) && !(sr1 & I2C_SR1_AF)) {  /* master reading from us */
        I2C3->DR = win_read(i2c_off);
        i2c_off++;
    }

    if (sr1 & I2C_SR1_STOPF) {                /* STOP: read SR1 then write CR1 */
        (void)I2C3->SR1;
        I2C3->CR1 |= I2C_CR1_PE;
        i2c_stops++;
        /* i2c_off is intentionally KEPT -- the offset-write/STOP/read sequence
         * depends on the offset surviving the STOP (docs/protocol.md:76). */
    }
}

void I2C3_ER_IRQHandler(void)
{
    /* AF is a normal end-of-read NACK from the master; just clear the errors. */
    I2C3->SR1 &= ~(I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR);
}

/* ======================================================================== *
 *  USART6 Linux flow packet (/dev/ttyS1) -- docs/protocol.md:775, usart.md:50.
 *  USART6 PC6(TX)/PC7(RX) AF8 streams a 0x1028-byte IQ+telemetry packet to the
 *  R16 every ~35 ms via DMA2 Stream 6, CRC32-stamped by the STM32 hardware
 *  engine. Baud = 1,500,000 (on-wire rate). DMA2 S6 + USART6 + CRC are unused
 *  by the RX engine (which is on DMA2 S0/S2 + DMA1), so there is no collision.
 *  IQ is zero-filled in this first merged build (RX audio goes out the BASE's
 *  own codec, already proven); live IQ + telemetry fold in next.
 * ======================================================================== */
#define FLOW_BAUD        1500000u       /* on-wire rate, confirmed; usart.md:82 */
#define FLOW_MAGIC       0xAA5555AAu
#define FLOW_BYTES       0x1028u        /* total DMA length (4136 bytes)         */
#define FLOW_CRC_WORDS   0x409u         /* words fed to the CRC engine (4132 B)  */
#define FLOW_CRC_OFF     0x1024u        /* byte offset of the CRC32 field        */
#define FLOW_INFO_OFF    0x1014u        /* byte offset of flow_info_t            */
#define FLOW_PERIOD_MS   35u            /* OEM typical packet interval           */

/* flow_info_t occupies the OEM "0x1014 reserved" 12 bytes. Byte-identical to
 * X6100Control low/flow.h + Georgy's x6100_base_patch comm.h, so the OpenX6100
 * GUI (which reads the SAME struct) decodes our bitfields exactly. Only read
 * when the GUI sees base rev>=8 (our fw-id ",r8"). */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t lo_freq;             /* IQ center (antenna Hz) for absolute panadapter */
    uint8_t  flow_fmt;            /* 0 = fp32 (512 cplx), 1 = bf16 (1024)           */
    uint8_t  flow_seq_n: 4;       /* block index within one collected buffer        */
    uint8_t  flow_seq_total: 4;   /* number of blocks per buffer                    */
    uint8_t  vary_freq: 1;        /* 1 => lo_freq changed vs the previous packet    */
    uint8_t  fft_dec: 3;          /* flow decimation exponent (0 = full 100 kHz)    */
    uint16_t audio_in_lvl_db: 7;  /* TX audio-in level, 0..127 (+127 dB offset)     */
    uint32_t _pad1: 5;
    uint32_t _pad2;
} flow_info_t;
_Static_assert(sizeof(flow_info_t) == 12u, "flow_info_t must be 12 bytes (hkey@0x1020)");

/* 4136-byte packet, DMA-addressable SRAM, 4-byte aligned for the CRC engine. */
static union {
    uint8_t  b[FLOW_BYTES];
    uint32_t w[FLOW_BYTES / 4u];          /* 0x40A = 1034 words */
} flow __attribute__((aligned(4)));

/* ======================================================================== *
 *  External hardware-key input (hkey) -- hand-mic / keypad buttons.
 *  Faithful reimplementation of the OEM path (x6100-base docs/hkey.md,
 *  FUN_08030334): TIM4 CH1 PWM-input capture on PD12/AF2 measures each
 *  encoded pulse; the decoder classifies the ratio CCR1/CCR2 into 3 symbol
 *  classes and reads a frame = 7x preamble(sym0) + 1x delimiter(sym2) + 39
 *  payload bits (LSB first), split 20 bits -> w1, 19 bits -> w2. It validates
 *  the redundant bits 12..19 across w1/w2, then publishes
 *    hkey = ((w1>>6)&0xF0) | ((w1&0x7FFFF)>>15)
 *  into the flow packet word at 0x1020. The code is cleared to 0 ~134 ms
 *  after the last capture (OEM 200 service-ticks). The GUI (radio.c
 *  hkey_put(pack->hkey)) maps the 8-bit code to X6100_HKEY_* actions. PD12
 *  AF2 is already set in common/bsp.c. TIM4 is on APB1 (125 MHz timer clock).
 * ======================================================================== */
#define HKEY_TIMEOUT_MS  135u            /* OEM 200 * 672 us ~= 134 ms            */

static volatile uint8_t  hkey_code;      /* published key code -> flow 0x1020     */
static volatile uint16_t hkey_idle_ms;   /* ms since the last TIM4 capture        */

/* Decoder state -- TIM4 ISR context only. */
static int      hk_state;                /* 0..6 preamble, 7 expect delimiter, 8 payload */
static int      hk_bit;                  /* payload bit index 0..39               */
static uint32_t hk_w1, hk_w2;            /* 20-bit + 19-bit accumulators (LSB first) */

static void hkey_decode(uint32_t ccr1, uint32_t ccr2)
{
    int sym = -1;                                     /* -1 = unrecognized        */
    if (ccr1 && ccr2) {
        float r = (float)ccr1 / (float)ccr2;          /* period / active interval */
        if      (r >= 1.7f  && r <= 2.3f)        sym = 0;  /* preamble / payload 0 */
        else if (r >= 2.5500002f && r <= 3.4499998f) sym = 1;  /* payload 1        */
        else if (r >= 4.25f && r <= 5.75f)       sym = 2;  /* frame delimiter      */
    }
    if (hk_state < 7) {                               /* collect 7 preamble 0s    */
        if (sym == 0) { hk_state++; return; }
    } else if (hk_state == 7) {                       /* expect the delimiter     */
        if (sym == 2) { hk_bit = 0; hk_w1 = 0; hk_w2 = 0; hk_state = 8; return; }
    } else if (sym == 0 || sym == 1) {                /* payload bit (LSB first)  */
        if (hk_bit < 20) {
            hk_w1 |= (uint32_t)sym << hk_bit;
            hk_bit++;
            return;
        }
        if (hk_bit < 39) {
            hk_w2 |= (uint32_t)sym << (hk_bit - 20);
            hk_bit++;
            if (hk_bit != 39) return;
        }
        /* 39 bits in: validate the redundant bits 12..19, then publish. */
        if (((hk_w2 ^ hk_w1) & 0x000FF000u) == 0u)
            hkey_code = (uint8_t)(((hk_w1 >> 6) & 0xF0u) | ((hk_w1 & 0x7FFFFu) >> 15));
    }
    hk_state = 0;                                     /* mismatch or frame end    */
}

void TIM4_IRQHandler(void)
{
    if (TIM4->SR & TIM_SR_CC1IF) {
        uint32_t ccr1 = TIM4->CCR1;                   /* period (TI1FP1 reset)    */
        uint32_t ccr2 = TIM4->CCR2;                   /* active interval          */
        TIM4->SR = (uint32_t)~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_UIF);
        hkey_idle_ms = 0;                             /* refresh the release timer */
        hkey_decode(ccr1, ccr2);
    } else {
        TIM4->SR = (uint32_t)~TIM_SR_UIF;
    }
}

/* TIM4 CH1 PWM input on PD12/AF2: PSC 8 -> 72 ns tick, ARR 0xFFFF, IC filter
 * 0xF; IC1 = TI1 rising -> CCR1 (period), IC2 = TI1 falling -> CCR2 (active);
 * slave reset mode on TI1FP1; CC1 interrupt. */
static void hkey_tim4_init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
    TIM4->CR1   = 0u;
    TIM4->PSC   = 8u;                                 /* 125 MHz / 9 = 13.9 MHz    */
    TIM4->ARR   = 0xFFFFu;
    TIM4->CCMR1 = (0x1u << TIM_CCMR1_CC1S_Pos) | (0xFu << TIM_CCMR1_IC1F_Pos)
                | (0x2u << TIM_CCMR1_CC2S_Pos) | (0xFu << TIM_CCMR1_IC2F_Pos);
    TIM4->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P;  /* IC1 rising, IC2 falling */
    TIM4->SMCR  = (0x5u << TIM_SMCR_TS_Pos) | (0x4u << TIM_SMCR_SMS_Pos);  /* TI1FP1, reset */
    TIM4->SR    = 0u;
    TIM4->DIER  = TIM_DIER_CC1IE;
    NVIC_SetPriority(TIM4_IRQn, 7);                   /* below DSP (5) and I2C3 (6) */
    NVIC_EnableIRQ(TIM4_IRQn);
    TIM4->CR1   = TIM_CR1_CEN;
}

/* Release timeout: called once per main-loop ms. Clears hkey_code ~134 ms after
 * the last capture (the keypad re-sends frames continuously while held). */
static void hkey_tick_ms(void)
{
    if (hkey_code && hkey_idle_ms < 0xFFFFu && ++hkey_idle_ms >= HKEY_TIMEOUT_MS)
        hkey_code = 0u;
}

/* STM32 hardware CRC32 (poly 0x04C11DB7, init 0xFFFFFFFF, non-reflected): write
 * 1 to CRC_CR to reset DR, feed n words, read DR (docs/protocol.md:825). */
static uint32_t crc32_hw(const volatile uint32_t *w, uint32_t n)
{
    CRC->CR = CRC_CR_RESET;
    for (uint32_t i = 0; i < n; i++) CRC->DR = w[i];
    return CRC->DR;
}

/* Fill the packet and stamp its CRC.
 *   - IQ (0x0004..0x1003): 512 complex f32 of the live 100 kHz raw RX I/Q,
 *     snapshotted by the DSP ISR (rx_flow_iq_copy). The x6100_gui consumer maps
 *     these across FULL_BW_HZ = 100 kHz for the waterfall. If no fresh block is
 *     ready this interval the region keeps its previous block (OEM behavior), so
 *     the waterfall simply does not advance rather than tearing.
 *   - 0x1004 flags: ext-power present (bit3). Battery flags (charging) and the
 *     battery telemetry (vbat/batcap) stay 0 pending the MAX17205 BMS increment.
 *   - 0x100C ext voltage, tenths of a volt: live from ADC2/PA6 (external_voltage_mv,
 *     * 0.01). Reads ~0 with no analog front end (emulator); real on hardware. */
static void flow_build(void)
{
    flow.w[0] = FLOW_MAGIC;                         /* 0x0000 magic              */
    rx_flow_iq_copy(&flow.b[0x0004]);               /* 0x0004 live 100 kHz IQ    */

    /* Telemetry: external supply (ADC2/PA6) + battery (MAX17205 via g_bat_*,
     * refreshed ~1 Hz from the main loop). The GUI power widget (clock.c) shows
     * the battery icon + vbat always, the battery % when on battery (vext<3 V),
     * the ext voltage when plugged in, and a charging bolt when flag.charging. */
    uint32_t vext_dV = (uint32_t)(external_voltage_mv() * 0.01f + 0.5f);
    if (vext_dV > 255u) vext_dV = 255u;
    int ext_present = (vext_dV >= 50u);             /* >= 5.0 V on the DC jack   */
    uint32_t vbat_dV = (uint32_t)(g_bat_mv * 0.01f + 0.5f);
    if (vbat_dV > 255u) vbat_dV = 255u;
    uint8_t batcap = bat_cap_by_voltage(g_bat_mv);
    /* Charging = external present AND the MAX17205 reports current flowing INTO
     * the pack. Sign convention confirmed on hardware 2026-11-11 (battery-only
     * run: AvgCurrent ~ -549 raw while discharging), so positive raw = charge.
     * Threshold ~+50 mA (x0.15625 mA/LSB) rejects the near-zero full/trickle
     * state, so a topped-off pack on external power reads "not charging". */
    int charging = ext_present && (g_bat_cur_raw > BAT_CHG_RAW_MIN);

    uint32_t flags = 0u;
    if (ext_present) flags |= 0x00000008u;          /* bit3 vext present         */
    if (charging)    flags |= 0x00000010u;          /* bit4 charging active      */
    flow.w[0x401]  = flags;                         /* 0x1004 flags              */
    flow.b[0x100C] = (uint8_t)vext_dV;              /* 0x100C ext voltage (dV)   */
    flow.b[0x100D] = (uint8_t)vbat_dV;              /* 0x100D battery voltage(dV)*/
    flow.b[0x100E] = batcap;                        /* 0x100E battery capacity % */

    /* flow_info (rev>=8): lo_freq = the actual LO center in antenna Hz
     * (= station + mode LO offset, == apply_tune's ftuned) so the GUI positions
     * the 100 kHz panadapter absolutely; fp32 / no flow decimation this build.
     * vary_freq flags the packet where the center moved (retune). fft_dec stays
     * 0 (full 100 kHz span); the reg15-commanded iirdecim zoom is a follow-on. */
    static uint32_t prev_lo_freq;
    uint32_t lo_freq = (uint32_t)((int64_t)rx_cur_station_hz + rx_lo_offset_hz);
    flow_info_t *fi = (flow_info_t *)&flow.b[FLOW_INFO_OFF];
    fi->lo_freq        = lo_freq;
    fi->flow_fmt       = rx_flow_iq_block_fmt();     /* 0 fp32 / 1 bf16, matches IQ */
    fi->flow_seq_n     = 0u;                         /* one self-contained block  */
    fi->flow_seq_total = 1u;
    fi->vary_freq      = (lo_freq != prev_lo_freq) ? 1u : 0u;
    fi->fft_dec        = rx_flow_iq_block_fft_dec(); /* matches the IQ in this packet */
    fi->audio_in_lvl_db = 0u;                        /* RX: no TX audio input     */
    fi->_pad1          = 0u;
    fi->_pad2          = 0u;
    prev_lo_freq       = lo_freq;

    flow.w[0x408] = hkey_code;                      /* 0x1020 hardware key code  */
    flow.w[FLOW_CRC_OFF / 4u] = crc32_hw(flow.w, FLOW_CRC_WORDS);
}

static void usart6_flow_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC | LL_AHB1_GRP1_PERIPH_DMA2
                             | LL_AHB1_GRP1_PERIPH_CRC);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);

    /* PC6 = USART6_TX, PC7 = USART6_RX, AF8 (gpio.md:89), high speed. */
    M_AF(GPIOC, 6); M_PP(GPIOC, 6); SPD_HI(GPIOC, 6);
    LL_GPIO_SetAFPin_0_7(GPIOC, LL_GPIO_PIN_6, LL_GPIO_AF_8);
    M_AF(GPIOC, 7); SPD_HI(GPIOC, 7);
    LL_GPIO_SetAFPin_0_7(GPIOC, LL_GPIO_PIN_7, LL_GPIO_AF_8);

    /* USART6 on APB2 (PCLK2). BRR = round(PCLK2 / baud) in OVER16 mode. */
    USART6->CR1 = 0;
    USART6->BRR = (PCLK2_HZ + FLOW_BAUD / 2u) / FLOW_BAUD;
    USART6->CR3 = USART_CR3_DMAT;                    /* TX served by DMA          */
    USART6->CR1 = USART_CR1_UE | USART_CR1_TE;       /* 8-N-1, transmitter on     */

    /* DMA2 Stream 6, channel 5: memory->peripheral, byte-wide, MINC, one-shot. */
    DMA2_Stream6->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream6->CR & DMA_SxCR_EN) { }
    DMA2_Stream6->PAR = (uint32_t)&USART6->DR;
    DMA2_Stream6->CR  = (5u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
}

/* Kick off one 4136-byte DMA burst (a full packet at 1.5 Mbaud is ~28 us). */
static void flow_send(void)
{
    while (DMA2_Stream6->CR & DMA_SxCR_EN) { }
    DMA2->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6
                | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;
    DMA2_Stream6->M0AR = (uint32_t)flow.b;
    DMA2_Stream6->NDTR = FLOW_BYTES;
    DMA2_Stream6->CR  |= DMA_SxCR_EN;
}

/* ======================================================================== *
 *  Step 4: OEM control-word -> RX-engine translation (VFO + RX audio first).
 * ======================================================================== */

/* cwr() is defined with the codec startup below; forward-declare it so the
 * rxvol path can drive the HPR analog-gain register. */
static int cwr(uint8_t addr, const uint8_t *buf, uint32_t n);

/* Apply the OEM rxvol transfer (reg14 bits 7:0; audio_control_register_mapping.csv,
 * BASE 1.1.9 FUN_0802c7d0). Range 0..59, net loudness ~ (V-30) dB:
 *   V=0      -> mute HPR (page1 R0x11 = 0x40)
 *   V=1..29  -> HPR analog 0 dB, digital scalar 10^((V-30)*0.05)   (-29..-1 dB)
 *   V=30..59 -> HPR analog (V-30) dB (R0x11 = 0x00..0x1D), digital scalar 1.0
 * Attenuation is digital and makeup gain is analog HPR, preserving digital
 * headroom exactly as the OEM. HPR -> LM386 -> internal speaker; HPL (the Linux
 * capture path) is left untouched. The digital scalar is rx_audio_gain, so with
 * the AGC on it multiplies the AGC-normalized level. */
static void apply_rxvol(uint32_t v)
{
    v &= 0xFFu;
    if (v > 59u) v = 59u;                               /* clamp to the OEM range */
    uint8_t hpr;                                        /* page1 R0x11: bit6 mute | signed dB */
    float   dig;
    if (v == 0u)       { hpr = 0x40u;              dig = 0.0f; }                 /* mute   */
    else if (v <= 29u) { hpr = 0x00u;              dig = powf(10.0f, (float)((int)v - 30) * 0.05f); }
    else               { hpr = (uint8_t)(v - 30u); dig = 1.0f; }                 /* +dB HPR */

    uint8_t p1[]   = {0x00, 0x01};                      /* select page 1          */
    uint8_t whpr[] = {0x11, hpr};                       /* HPR driver gain / mute */
    cwr(ADDR_CODEC, p1, sizeof p1);
    cwr(ADDR_CODEC, whpr, sizeof whpr);
    rx_audio_gain = dig;
}

/* OEM reg15 RF-gain low byte -> DSP input scalar (docs/protocol.md reg15, BASE
 * 1.1.9 FUN_08023dd8). The GUI's RF-GAIN knob is 0..100 (default 63):
 *   rfg 0..36   -> scalar 10^((0.5*rfg - 18)*0.05)   (a progressive cut, to -18 dB)
 *   rfg 37..255 -> scalar 1.0  (the six-bit stepped RF network, which this RX-only
 *                  FW holds at its power-on state, is the gain element in-range)
 * So RF-GAIN below 37 digitally attenuates the front end (audio + panadapter); at
 * or above 37 the DSP scalar is unity and the hardware network -- not modelled
 * here -- would carry it. Faithful for the cut region; the in-range steps are a
 * no-op by design (we do not drive that network in RX-only). */
static float rfg_to_scalar(uint32_t rfg)
{
    rfg &= 0xFFu;
    if (rfg <= 36u)
        return powf(10.0f, (0.5f * (float)rfg - 18.0f) * 0.05f);
    return 1.0f;
}

/* Map the OEM mode enum (0..7 = LSB,LSB_DIG,USB,USB_DIG,CW,CWR,AM,NFM;
 * docs/protocol.md:278) onto phase3-rx's RX_MODE set. RX-only: the digital
 * sub-modes collapse to their sideband, CW/CWR use their conventional sideband,
 * and NFM (no FM demod yet) falls back to AM. */
static int oem_mode_to_rx(uint32_t m)
{
    switch (m & 7u) {
    case 0:  return RX_MODE_LSB;   /* LSB     */
    case 1:  return RX_MODE_LSB;   /* LSB_DIG */
    case 2:  return RX_MODE_USB;   /* USB     */
    case 3:  return RX_MODE_USB;   /* USB_DIG */
    case 4:  return RX_MODE_USB;   /* CW  -> USB side */
    case 5:  return RX_MODE_LSB;   /* CWR -> LSB side */
    case 6:  return RX_MODE_AM;    /* AM      */
    default: return RX_MODE_AM;    /* NFM -> AM placeholder */
    }
}

/* VERIFIED OEM per-mode LO adjustment (frequency_control.md:47): modes 0..3
 * (LSB,LSB_DIG,USB,USB_DIG) = 0; 4 (CW) = -CW tone; 5 (CWR) = +CW tone; 6 (AM)
 * = -100 Hz; 7 (NFM) = -200 Hz. CW/CWR need the user-configured CW tone, which
 * is not yet carried in a control word, so they tune zero-beat (0) for now; the
 * other five match the OEM exactly. Applied as Ftuned = Fvfo + offset. */
static int32_t oem_mode_lo_offset(uint32_t m)
{
    switch (m & 7u) {
    case 6:  return -100;   /* AM  */
    case 7:  return -200;   /* NFM */
    default: return 0;      /* LSB/LSB_DIG/USB/USB_DIG; CW/CWR (tone TBD) */
    }
}

/* Translate the OEM 56-word control file into RX-engine actions. Called every
 * 1 ms from the main loop; applies only on change.
 *   reg13 bit0   -> active VFO (0=A,1=B), selects the freq/mode word pair
 *   reg1 / reg7  -> VFO A/B frequency (u32 Hz)   -> apply_tune()
 *   reg4 / reg10 -> VFO A/B mode (enum)          -> rx_mode + LO offset
 *   reg14 low B  -> rxvol (0..59)                -> apply_rxvol() OEM transfer
 *   reg5 / reg11 -> VFO A/B AGC mode (0 off,1 slow,2 fast,3 auto)  -> rx_agc_set
 *   reg28        -> agctime (ms, GUI-folded per mode)  -> AGC release constant
 *   reg27 bit12  -> agchang (hang enable)              -> AGC release hang
 *   reg15 low B  -> rfg (0..100 RF gain)   -> rx_rf_gain input scalar (OEM xfer)
 * Freq 0 is ignored so a freshly-zeroed control file does not retune away from
 * the power-on default until Linux writes a real frequency. Still TODO (separate
 * increment): att/pre (reg2/3/8/9 hardware), NR (reg23), notch (reg24), RX
 * filter edges (reg44-47), RIT (reg42). */
static void control_scan_apply(void)
{
    /* An all-zero control file means Linux has not published it yet (power-on
     * BSS state), NOT "set everything to zero" -- a real OEM control file always
     * carries a VFO frequency. Until one appears, hold the firmware power-on
     * defaults (AM, RX_TUNED_HZ, AGC on, silent) so standalone RX is unaffected
     * and the zeroed mode word does not clobber AM into LSB at boot. */
    if (ctrl.w[1] == 0u && ctrl.w[7] == 0u)
        return;

    static uint32_t last_v13  = 0xFFFFFFFFu;
    static uint32_t last_freq = 0u;
    static uint32_t last_mode = 0xFFFFFFFFu;
    /* last_vol seeds to 0 (not "unknown"): an untouched rxvol of 0 must NOT call
     * apply_rxvol on gate-open, so a frequency write stays volume-neutral (leaves
     * the HPR driver at its startup state). rx_audio_gain defaults to 0 = silent,
     * and the first real VOL write drives both HPR gain and the digital scalar. */
    static uint32_t last_vol  = 0u;
    /* AGC sentinel packs {mode, agctime, agchang} so any one changing re-applies;
     * seeds to "unknown" so the first real control file sets the AGC (the FW boots
     * with the AGC on by default, and a real file carries agc=auto, see below). */
    static uint32_t last_agc  = 0xFFFFFFFFu;
    static uint32_t last_rfg  = 0xFFFFFFFFu;             /* reg15 RF-gain low byte  */

    uint32_t v13  = ctrl.w[13];
    unsigned b    = (unsigned)(v13 & 1u);                 /* 0=VFO A, 1=VFO B */
    uint32_t freq = b ? ctrl.w[7]  : ctrl.w[1];
    uint32_t mode = (b ? ctrl.w[10] : ctrl.w[4]) & 7u;
    uint32_t vol  = ctrl.w[14] & 0xFFu;                   /* rxvol low byte */
    uint32_t agcm = (b ? ctrl.w[11] : ctrl.w[5]) & 3u;    /* AGC mode 0..3 (per VFO) */
    uint32_t agct = ctrl.w[28];                           /* agctime (ms, raw)       */
    uint32_t agch = (ctrl.w[27] >> 12) & 1u;              /* agchang (hang enable)   */
    uint32_t rfg  = ctrl.w[15] & 0xFFu;                   /* RF gain (0..100)        */
    /* Fold the AGC words into one change token (mode | time | hang-flag). */
    uint32_t agc  = agcm | ((agct & 0x7FFu) << 2) | (agch << 13);

    /* Active-VFO switch: force freq+mode+AGC to re-apply from the newly selected VFO. */
    if (v13 != last_v13) {
        last_v13  = v13;
        last_freq = 0u;
        last_mode = 0xFFFFFFFFu;
        last_agc  = 0xFFFFFFFFu;
        bsp_uprintf("CTRL reg13=0x%08lX -> active VFO %c\r\n",
                    (unsigned long)v13, b ? 'B' : 'A');
    }

    if (mode != last_mode) {
        last_mode = mode;
        rx_mode = oem_mode_to_rx(mode);
        rx_lo_offset_hz = oem_mode_lo_offset(mode);   /* VERIFIED OEM per-mode LO offset */
        apply_tune(rx_cur_station_hz);                    /* re-apply with new offset */
        bsp_uprintf("CTRL mode[%u]=%lu -> %s (lo_offset %ld Hz)\r\n",
                    b ? 10u : 4u, (unsigned long)mode,
                    rx_mode == RX_MODE_AM ? "AM" : rx_mode == RX_MODE_LSB ? "LSB" : "USB",
                    (long)rx_lo_offset_hz);
    }

    if (freq != 0u && freq != last_freq) {
        last_freq = freq;
        apply_tune(freq);
        bsp_uprintf("CTRL freq[%u]=%lu Hz -> tuned\r\n", b ? 7u : 1u, (unsigned long)freq);
    }

    if (vol != last_vol) {
        last_vol = vol;
        apply_rxvol(vol);                       /* faithful OEM rxvol transfer */
        bsp_uprintf("CTRL rxvol=%lu (0..59) -> %s, digital gain %ld/1000\r\n",
                    (unsigned long)vol,
                    vol == 0u ? "HPR MUTE" : vol <= 29u ? "HPR 0dB (digital atten)"
                                                        : "HPR analog +gain",
                    (long)(rx_audio_gain * 1000.0f));
    }

    /* AGC: reg5/11 mode (0 off,1 slow,2 fast,3 auto), reg28 agctime (ms), reg27
     * bit12 agchang. Mode 0 disables the AGC -> fixed rx_audio_gain / RF-gain path.
     * The paired GUI already folds the AUTO per-mode timing into agctime (radio.c
     * update_agc_time: slow/off 1000, fast 100, auto 100/500/1000 by mode), so we
     * take agctime straight as the release (decay) constant and keep the firmware
     * attack/target; hang is on when agchang is set. */
    if (agc != last_agc) {
        last_agc = agc;
        if (agcm == 0u) {
            rx_agc_set(0, 0, 0, 0);               /* OFF -> fixed-gain fallback */
            bsp_uprintf("CTRL agc[%u]=OFF -> fixed-gain path\r\n", b ? 11u : 5u);
        } else {
            uint16_t dcy = (uint16_t)(agct & 0x7FFu);     /* GUI writes raw ms (<=1000) */
            if (dcy < 20u)   dcy = (uint16_t)RX_AGC_DEF_DCY_MS;    /* unset -> default */
            if (dcy > 2000u) dcy = 2000u;
            uint16_t hang = agch ? (uint16_t)RX_AGC_HANG_MS : 0u;
            rx_agc_set(RX_AGC_DEF_TARGET_CDB, RX_AGC_DEF_ATK_MS, dcy, hang);
            bsp_uprintf("CTRL agc[%u]=%s -> ON (release %u ms, hang %u ms)\r\n",
                        b ? 11u : 5u,
                        agcm == 1u ? "SLOW" : agcm == 2u ? "FAST" : "AUTO",
                        (unsigned)dcy, (unsigned)hang);
        }
    }

    /* reg15 low byte = OEM RF gain (0..100). Below 37 it digitally cuts the front
     * end (audio + panadapter floor); at/above 37 the DSP scalar is unity. */
    if (rfg != last_rfg) {
        last_rfg = rfg;
        float sc = rfg_to_scalar(rfg);
        rx_rf_gain = sc;
        bsp_uprintf("CTRL rfg=%lu -> input scalar %ld/1000 (%s)\r\n",
                    (unsigned long)rfg, (long)(sc * 1000.0f),
                    rfg <= 36u ? "DSP cut" : "unity (hw network)");
    }

    /* reg15 (RF gain / TX power) bits 16..19 = panadapter zoom fft_dec (0..3);
     * the GUI maps the waterfall span to 100 kHz / 2^fft_dec. */
    static uint32_t last_fft_dec = 0xFFFFFFFFu;
    uint32_t fft_dec = (ctrl.w[15] >> 16) & 0xFu;
    if (fft_dec > 3u) fft_dec = 3u;
    if (fft_dec != last_fft_dec) {
        last_fft_dec = fft_dec;
        rx_flow_set_fft_dec((uint8_t)fft_dec);
        bsp_uprintf("CTRL fft_dec=%lu -> flow IQ span %lu kHz\r\n",
                    (unsigned long)fft_dec, 100UL >> fft_dec);
    }

    /* reg18 (flow_fm_emp) bit0 = flow IQ format: 0 fp32 (512 cplx), 1 bf16
     * (1024 cplx = 2x waterfall resolution). bit1 (fm_emp) is FM de-emphasis. */
    static uint32_t last_fmt = 0xFFFFFFFFu;
    uint32_t fmt = ctrl.w[18] & 1u;
    if (fmt != last_fmt) {
        last_fmt = fmt;
        rx_flow_set_fmt((uint8_t)fmt);
        bsp_uprintf("CTRL flow_fmt=%s -> %u complex samples\r\n",
                    fmt ? "bf16" : "fp32", fmt ? 1024u : 512u);
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
        g_bat_mv = (float)batt * 1.25f;              /* seed before the 1 Hz poll */
        g_bat_cur_raw = avgcur;
        bsp_uprintf("  RepSOC=%lu%%  Batt=%lu mV (cap=%u%% by LUT)  Temp=%ld C  AvgCurrent=%ld (x0.15625)\r\n",
                    (unsigned long)(repsoc * 39UL / 10000UL),       /* x0.0039 */
                    (unsigned long)(batt * 125UL / 100UL),          /* x1.25 mV */
                    (unsigned)bat_cap_by_voltage((float)batt * 1.25f),
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

    fwid_init();            /* "V1.1.9...,r8" fw-id -> GUI sees base rev 8 (flow_info) */
    i2c3_slave_init();
    usart6_flow_init();
    hkey_tim4_init();       /* PD12/TIM4 PWM-input capture -> flow hkey (keypad) */
    bsp_uprintf("  hkey: TIM4 CH1 PWM-input on PD12/AF2 -> flow 0x1020 (hand-mic/keypad)\r\n");
    bsp_uprintf("  I2C3 OEM control window @ 0x72 (PA8/PC9): 56-word control file "
                "(reg1/7 freq, reg4/10 mode, reg13 active-VFO, reg14 rxvol, "
                "reg5/11 agc, reg27/28 agc-hang/time, reg15 rfgain), "
                "fw-id read@0, reg55 handshake @0x00DC\r\n");
    bsp_uprintf("  USART6 flow @ /dev/ttyS1: PC6 TX AF8, %lu baud, DMA2 S6, %u B/%u ms\r\n",
                (unsigned long)FLOW_BAUD, (unsigned)FLOW_BYTES, (unsigned)FLOW_PERIOD_MS);
    bsp_uprintf("  reg55 boot handshake: TRACKED not gated (standalone RX preserved)\r\n");
    bsp_uprintf("  LO offset default = %ld Hz; mode = AM\r\n", (long)rx_lo_offset_hz);
    bsp_uprintf("  initial TUNE=%lu Hz, VOLUME=0.0 (silent until Linux writes rxvol); "
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
    bsp_uprintf("phase4-linux running: RX live (listen for WDOV) + OEM I2C3 window "
                "+ USART6 flow; control via /dev/i2c-0 @ 0x72\r\n");

    uint32_t ms = 0, last_banks = 0;
    /* Cycles available per 1.28 ms bank (128 I/Q samples @ 100 kHz) for the CPU%
     * gauge. The DSP runs in the DMA ISR, so this is its worst-case bank budget. */
    const uint32_t budget_cyc = (HCLK_HZ / 100000u) * I2S_BANK_FRAMES;
    uint32_t flow_ms = 0, last_reg55 = 0;
    for (;;) {
        bsp_delay_ms(1);
        control_scan_apply();   /* OEM control file -> VFO + RX audio */
        hkey_tick_ms();         /* clear hkey ~134 ms after the last keypad frame */
        {   /* Log each keypad press (rising edge to a non-zero code). */
            static uint8_t last_hk;
            uint8_t hk = hkey_code;
            if (hk != last_hk) {
                if (hk) bsp_uprintf("hkey press: 0x%02X\r\n", hk);
                last_hk = hk;
            }
        }

        /* Keep the ADC2 telemetry IIR (external-supply voltage) converging so the
         * flow packet's ext-voltage byte is live. Cheap (3-channel filter step). */
        adc2_filter_step((const uint16_t *)adc2_buf);

        /* Emit the USART6 flow packet at the OEM ~35 ms cadence. */
        if (++flow_ms >= FLOW_PERIOD_MS) {
            flow_ms = 0;
            flow_build();
            flow_send();
        }

        /* Report the reg55 boot-ready transition (tracked, not gated). */
        {
            uint32_t r55 = ctrl.w[REG_HANDSHAKE_INDEX];
            if ((r55 & HANDSHAKE_READY_BIT) && !(last_reg55 & HANDSHAKE_READY_BIT))
                bsp_uprintf("CTRL reg55=0x%08lX: Linux boot-ready\r\n", (unsigned long)r55);
            last_reg55 = r55;
        }

        if ((++ms % 1000u) == 0u) {
            bat_telemetry_poll();   /* refresh g_bat_mv/g_bat_cur_raw (MAX17205, ~1 Hz) */
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
            {   /* BMS status mirroring the flow packet's battery/power fields. */
                long bat_mv_i = (long)g_bat_mv;
                long ext_mv_i = (long)external_voltage_mv();
                unsigned cap  = bat_cap_by_voltage(g_bat_mv);
                int ext_present = (ext_mv_i >= 5000);
                bsp_uprintf("  BMS: Vbat=%ld mV cap=%u%% Vext=%ld mV ext=%d charging=%d "
                            "cur=%ld mA\r\n", bat_mv_i, cap, ext_mv_i, ext_present,
                            ext_present && (cap < 100u),
                            (long)((g_bat_cur_raw * 15625L) / 100000L));
            }
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
