/*
 * x6100-bbfw -- Phase 2b-HW: Phase 2a bring-up + live I2S sample transport
 * =======================================================================
 * HARDWARE test vehicle. This is the Phase 2a firmware (ADC/DAC/MAX17205 +
 * the full TLV320/CS4272 codec bring-up that makes the TLV320 the I2S clock
 * master, R0x1B=0x3C) followed by the Phase 2b I2S/DMA transport (i2s.c).
 * Only on real hardware is BCLK/WCLK present, so only here do the four DMA
 * double-buffers actually clock and the DMA1_Stream3 ISR advance bank_count.
 *
 * Safe throughout: PA + charger stay disabled; TX banks are zero-filled, so
 * both transmit I2S pairs are electrically inactive; the codec stays muted.
 * Receive-only. Expected on the bench: BCLK/WCLK lock on a scope and
 * bank_count climbing at ~781 Hz (128 frames / 100 kHz = 1.28 ms).
 *
 * (Original Phase 2a header follows.)
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

#define FW_VERSION "x6100-bbfw phase2b-hw 2026-10-30"

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

    /* ---- I2S sample transport (Phase 2b) ---- */
    bsp_uprintf("-- I2S sample transport --\r\n");
    bsp_uprintf("  DSP rates: raw=%lu primary=%lu audio=%lu alt=%lu Hz\r\n",
                (unsigned long)i2s_rate_raw, (unsigned long)i2s_rate_primary,
                (unsigned long)i2s_rate_audio, (unsigned long)i2s_rate_alt);
#ifdef PHASE2_DIAG
    i2s_clock_sniff();   /* no-scope: does the codec actually clock WCLK/BCLK? */
#endif
    i2s_config();
    i2s_start();
    bsp_uprintf("  4 DMA1 streams + I2S slave RX / ext TX enabled; "
                "DMA1_Stream3 IRQ armed\r\n");
    bsp_uprintf("  expect BCLK/WCLK present and bank_count climbing "
                "(~781/s at 100 kHz); TX banks zero -> inactive\r\n");
    bsp_uprintf("phase2b-hw running; safe state held (PA off, charger off, +5T off)\r\n");

    uint32_t ms = 0, last_banks = 0;
    for (;;) {
        bsp_delay_ms(1);
        if ((++ms % 1000u) == 0u) {
            uint32_t banks = i2s_bank_count;
            bsp_uprintf("alive t=%lu ms  banks=%lu (+%lu/s)  last_audio=[%ld,%ld] "
                        "last_iq=[%ld,%ld]\r\n",
                        (unsigned long)ms, (unsigned long)banks,
                        (unsigned long)(banks - last_banks),
                        (long)i2s_last_audio[0], (long)i2s_last_audio[1],
                        (long)i2s_last_iq[0], (long)i2s_last_iq[1]);
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
