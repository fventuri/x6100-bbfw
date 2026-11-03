/*
 * x6100-bbfw -- Phase 2a: analog + telemetry + codec/converter init
 * =================================================================
 * The analog/I2C part of Phase 2 (x6100-base/docs/implementation_readiness.md).
 * The four I2S DMA streams are Phase 2b (they depend on the external codec I2S
 * clock, which Renode cannot faithfully reproduce).
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

#define FW_VERSION "x6100-bbfw phase2a-analog 2026-10-30"

#define ADDR_MAX17205  0x36u
#define ADDR_CODEC     0x18u
#define ADDR_CS4272    0x10u
#define ADDR_AUX4A     0x4Au
#define ADDR_AUX4C     0x4Cu

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

int main(void)
{
    bsp_gpio_init_safe();
    bsp_clock_init_250mhz();
    bsp_systick_init_1ms();
    bsp_usart1_init();

    bsp_uprintf("\r\n=== %s ===\r\n", FW_VERSION);
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

    /* Read-back a few codec registers from the device model to confirm the
     * sequence landed (page-agnostic model: these registers are written once). */
    uint8_t r = 0;
    if (i2c_read(I2C2, ADDR_CODEC, 0x1B, &r, 1) == 0)
        bsp_uprintf("  codec readback R0x1B=0x%02X (expect 0x3C: 32-bit I2S, BCLK/WCLK out)\r\n", r);
    if (i2c_read(I2C2, ADDR_CODEC, 0x09, &r, 1) == 0)
        bsp_uprintf("  codec readback R0x09=0x%02X (expect 0x3F: HPL/HPR/LOL/LOR+mixers)\r\n", r);

    bsp_uprintf("phase2a complete; I2S/DMA deferred to phase 2b; safe state held\r\n");

    uint32_t ms = 0;
    for (;;) {
        bsp_delay_ms(1);
        if ((++ms % 1000u) == 0u)
            bsp_uprintf("alive t=%lu ms\r\n", (unsigned long)ms);
    }
}
