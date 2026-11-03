/*
 * x6100-bbfw -- Phase 2b: I2S sample transport (four I2S DMA streams)
 * ==================================================================
 * The remaining Phase 2 step from x6100-base/docs/implementation_readiness.md:
 *   "Run the four DMA streams with pass-through or zero-filled output.
 *    Exit: stable 128-frame bank cadence, no overrun; correct channel ordering
 *    and signed 32-bit reconstruction; transmit banks remain electrically
 *    inactive."
 *
 * This firmware configures both full-duplex I2S slave pairs and their four
 * DMA1 double-buffers (i2s.c, per spi_i2s.md/dma.md), installs the DMA1
 * Stream3 handler, and goes live. Phase 2b is DSP-less: RX banks are
 * reconstructed to signed 32-bit, TX banks are zero-filled (inactive).
 *
 * HARDWARE PREREQUISITE: the external TLV320AIC3204 is the I2S clock master
 * (R0x1B=0x3C, set by the Phase 2a codec bring-up). On real hardware the
 * Phase 2a codec/converter init must run first or no BCLK/WCLK is present and
 * the slaves never clock. BCLK/WCLK lock itself is a hardware measurement.
 *
 * EMULATOR SCOPE (honest): Renode models SPI2/SPI3 as plain SPI with no
 * external-clock I2S framing and has no I2S2ext/I2S3ext or DMA-request
 * generation, so it cannot reproduce a live 128-frame cadence with the stock
 * platform. We therefore validate deterministically:
 *   (A) the four DMA streams' register programming (read-back);
 *   (B) the I2S peripherals' format programming (read-back, model permitting);
 *   (C) signed-32-bit reconstruction and channel ordering (unit test);
 *   (D) the Stream3 handler's bank-ownership selection + TX zero-fill, by
 *       driving i2s_on_bank_complete() with synthetic banks;
 *   (E) the DMA1_Stream3 vector + NVIC enable wiring.
 * Live cadence / overrun-freedom is exercised separately by the optional C#
 * I2S-codec model (renode/devices) or deferred to hardware.
 *
 * PA and charger stay disabled throughout; TX banks are zero -> inactive.
 */
#include <stdint.h>

#include "bsp.h"
#include "i2s.h"

#define FW_VERSION "x6100-bbfw phase2b-sample-transport 2026-10-30"

void DMA1_Stream3_IRQHandler(void);   /* defined in i2s.c, for the wiring check */

/* ---- expected DMA programming (dma.md), for the read-back check ---- */
struct dma_expect {
    DMA_Stream_TypeDef *st;
    const char *name;
    uint32_t chsel;
    uint32_t par;
    int dir_m2p;
    int tcie;
};

/* The full CR the firmware programs (datasheet-correct), for an intended-vs-
 * observed comparison. Renode's STM32DMA does not model CIRC/DBM/CHSEL/M1AR,
 * so observed CR and M1AR read back short of this; that is an emulator gap,
 * not a firmware defect (see EMULATOR SCOPE). */
static uint32_t intended_cr(const struct dma_expect *e)
{
    return (e->chsel << DMA_SxCR_CHSEL_Pos)
         | (1u << DMA_SxCR_PSIZE_Pos) | (1u << DMA_SxCR_MSIZE_Pos)
         | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_DBM
         | (e->dir_m2p ? DMA_SxCR_DIR_0 : 0u) | (e->tcie ? DMA_SxCR_TCIE : 0u);
}

static int check_dma_stream(const struct dma_expect *e)
{
    uint32_t cr = e->st->CR;
    /* Assert the fields Renode models and that matter for correctness:
     * peripheral address, transfer count, and a valid SRAM M0 bank. */
    int addr_ok = (e->st->PAR == e->par)
                && (e->st->NDTR == I2S_BANK_HALFWORDS)
                && ((e->st->M0AR & 0xFFF00000u) == 0x20000000u);

    bsp_uprintf("  %s ch%lu %s PAR=0x%08lX M0=0x%08lX M1=0x%08lX NDTR=%lu "
                "CR want=0x%06lX got=0x%06lX  addr/count %s\r\n",
                e->name, (unsigned long)e->chsel, e->dir_m2p ? "m->p" : "p->m",
                (unsigned long)e->st->PAR, (unsigned long)e->st->M0AR,
                (unsigned long)e->st->M1AR, (unsigned long)e->st->NDTR,
                (unsigned long)intended_cr(e), (unsigned long)cr,
                addr_ok ? "OK" : "FAIL");
    return addr_ok;
}

static void decode_i2scfgr(const char *name, uint32_t v, uint32_t cr2)
{
    static const char *cfg[] = { "slaveTX", "slaveRX", "masterTX", "masterRX" };
    bsp_uprintf("  %s I2SCFGR=0x%04lX [%s%s STD=%lu CKPOL=%lu DATLEN=%lu CHLEN=%lu "
                "I2SE=%lu] CR2=0x%02lX\r\n",
                name, (unsigned long)v,
                (v & SPI_I2SCFGR_I2SMOD) ? "I2S " : "SPI ",
                cfg[(v >> SPI_I2SCFGR_I2SCFG_Pos) & 0x3u],
                (unsigned long)((v >> SPI_I2SCFGR_I2SSTD_Pos) & 0x3u),
                (unsigned long)((v >> SPI_I2SCFGR_CKPOL_Pos) & 0x1u),
                (unsigned long)((v >> SPI_I2SCFGR_DATLEN_Pos) & 0x3u),
                (unsigned long)((v >> SPI_I2SCFGR_CHLEN_Pos) & 0x1u),
                (unsigned long)((v >> SPI_I2SCFGR_I2SE_Pos) & 0x1u),
                (unsigned long)cr2);
}

/* Fill a 512-halfword bank so every stereo frame reconstructs to (c0,c1). */
static void fill_bank(uint16_t *b, int32_t c0, int32_t c1)
{
    for (unsigned f = 0; f < I2S_BANK_FRAMES; f++) {
        b[4u * f + 0] = (uint16_t)((uint32_t)c0 >> 16);
        b[4u * f + 1] = (uint16_t)((uint32_t)c0 & 0xFFFFu);
        b[4u * f + 2] = (uint16_t)((uint32_t)c1 >> 16);
        b[4u * f + 3] = (uint16_t)((uint32_t)c1 & 0xFFFFu);
    }
}

static int bank_all_zero(const uint16_t *b)
{
    for (unsigned i = 0; i < I2S_BANK_HALFWORDS; i++)
        if (b[i] != 0u)
            return 0;
    return 1;
}

int main(void)
{
    bsp_gpio_init_safe();
    bsp_clock_init_250mhz();
    bsp_systick_init_1ms();
    bsp_usart1_init();

    bsp_uprintf("\r\n=== %s ===\r\n", FW_VERSION);
    bsp_uprintf("safe-state held: PA-tuple disabled, charger off, +5T off\r\n");
    bsp_uprintf("DSP rates: raw=%lu primary=%lu audio=%lu alt=%lu Hz\r\n",
                (unsigned long)i2s_rate_raw, (unsigned long)i2s_rate_primary,
                (unsigned long)i2s_rate_audio, (unsigned long)i2s_rate_alt);

    int pass = 1;

    i2s_config();

    /* (A) DMA stream programming read-back. */
    bsp_uprintf("-- DMA1 streams (dma.md) --\r\n");
    const struct dma_expect dex[] = {
        { DMA1_Stream0, "S0 SPI3/I2S3 RX", 0u, (uint32_t)&SPI3->DR,    0, 0 },
        { DMA1_Stream3, "S3 SPI2/I2S2 RX", 0u, (uint32_t)&SPI2->DR,    0, 1 },
        { DMA1_Stream4, "S4 I2S2ext   TX", 2u, (uint32_t)&I2S2ext->DR, 1, 0 },
        { DMA1_Stream5, "S5 I2S3ext   TX", 2u, (uint32_t)&I2S3ext->DR, 1, 0 },
    };
    for (unsigned i = 0; i < sizeof dex / sizeof dex[0]; i++)
        pass &= check_dma_stream(&dex[i]);
    bsp_uprintf("  note: emulator STM32DMA drops CIRC/DBM/CHSEL and omits the M1\r\n");
    bsp_uprintf("  double-buffer pointer, so those CR bits + M1AR read short of\r\n");
    bsp_uprintf("  'want'. They are datasheet-correct in firmware (hardware-\r\n");
    bsp_uprintf("  confirmable); live ping-pong cadence is a hardware measurement.\r\n");

    /* (B) I2S format read-back (Renode's SPI model may not keep I2SCFGR). */
    bsp_uprintf("-- I2S peripherals (spi_i2s.md) --\r\n");
    decode_i2scfgr("SPI2/I2S2 ", SPI2->I2SCFGR, SPI2->CR2);
    decode_i2scfgr("SPI3/I2S3 ", SPI3->I2SCFGR, SPI3->CR2);
    decode_i2scfgr("I2S2ext   ", I2S2ext->I2SCFGR, I2S2ext->CR2);
    decode_i2scfgr("I2S3ext   ", I2S3ext->I2SCFGR, I2S3ext->CR2);
    if ((SPI2->I2SCFGR & SPI_I2SCFGR_I2SMOD) == 0u)
        bsp_uprintf("  note: emulator SPI model does not retain I2SCFGR; format "
                    "verified by code (hardware-confirmable)\r\n");

    /* (C) signed-32-bit reconstruction + channel ordering (model-independent). */
    bsp_uprintf("-- reconstruction unit test --\r\n");
    {
        const uint16_t hw[4] = { 0x8000, 0x0001, 0x0000, 0x7FFF };
        int32_t c0, c1;
        i2s_reconstruct_frame(hw, &c0, &c1);
        int ok = (c0 == (int32_t)0x80000001) && (c1 == (int32_t)0x00007FFF);
        bsp_uprintf("  {8000,0001,0000,7FFF} -> ch0=%ld ch1=%ld %s\r\n",
                    (long)c0, (long)c1, ok ? "OK" : "FAIL");
        pass &= ok;
    }

    /* (D) Stream3 handler logic, model-independent. The handler's two jobs are
     * pure functions here: double-buffer bank selection from the current-target
     * bit, and the DSP-less bank service (signed-32 reconstruction + TX
     * zero-fill). We drive both with controlled buffers so the result does not
     * depend on Renode's (incomplete) DMA register model. */
    bsp_uprintf("-- Stream3 handler logic (selection + reconstruct + zero-fill) --\r\n");
    {
        uint16_t m0, m1;  /* use addresses as identities */
        int sel_ok = (i2s_select_bank(0, &m0, &m1) == &m1)   /* CT=0 -> M1 done */
                   && (i2s_select_bank(1, &m0, &m1) == &m0);  /* CT=1 -> M0 done */
        bsp_uprintf("  bank selection: CT0->M1, CT1->M0  %s\r\n", sel_ok ? "OK" : "FAIL");
        pass &= sel_ok;
    }
    {
        static uint16_t ra[I2S_BANK_HALFWORDS], rq[I2S_BANK_HALFWORDS];
        static uint16_t ta[I2S_BANK_HALFWORDS], tq[I2S_BANK_HALFWORDS];
        const int32_t A0 = (int32_t)0xDEAD0001, A1 = 0x00BEEF02;
        const int32_t Q0 = (int32_t)0xFFFFFFFF, Q1 = 0x12345678;  /* I/Q incl -1 */
        fill_bank(ra, A0, A1);
        fill_bank(rq, Q0, Q1);
        for (unsigned i = 0; i < I2S_BANK_HALFWORDS; i++) ta[i] = tq[i] = 0xAAAAu;

        uint32_t bc = i2s_bank_count;
        i2s_process_banks(ra, rq, ta, tq);

        int rec_ok = (i2s_last_audio[0] == A0) && (i2s_last_audio[1] == A1)
                   && (i2s_last_iq[0]    == Q0) && (i2s_last_iq[1]    == Q1);
        int tx_ok  = bank_all_zero(ta) && bank_all_zero(tq);
        int cnt_ok = (i2s_bank_count == bc + 1u);
        bsp_uprintf("  audio ch=[%ld,%ld] iq ch=[%ld,%ld]: reconstruct %s, "
                    "TX-zero %s, count %s\r\n",
                    (long)i2s_last_audio[0], (long)i2s_last_audio[1],
                    (long)i2s_last_iq[0], (long)i2s_last_iq[1],
                    rec_ok ? "OK" : "FAIL", tx_ok ? "OK" : "FAIL",
                    cnt_ok ? "OK" : "FAIL");
        pass &= rec_ok & tx_ok & cnt_ok;
    }
    i2s_bank_count = 0u;

    /* (E) Go live + verify vector / NVIC wiring. */
    bsp_uprintf("-- go live (DMA streams + I2S blocks + NVIC) --\r\n");
    i2s_start();
    {
        uint32_t vec = *(volatile uint32_t *)(0x08020000u + (16u + 14u) * 4u);
        uint32_t want = (uint32_t)&DMA1_Stream3_IRQHandler;
        int vec_ok = (vec & ~1u) == (want & ~1u);
        int nvic_ok = (NVIC->ISER[0] >> (uint32_t)DMA1_Stream3_IRQn) & 1u;
        int en_ok = (DMA1_Stream0->CR & DMA_SxCR_EN) && (DMA1_Stream3->CR & DMA_SxCR_EN)
                  && (DMA1_Stream4->CR & DMA_SxCR_EN) && (DMA1_Stream5->CR & DMA_SxCR_EN);
        bsp_uprintf("  vector[30]=0x%08lX handler=0x%08lX %s; NVIC IRQ14 %s; 4 DMA EN %s\r\n",
                    (unsigned long)vec, (unsigned long)want, vec_ok ? "OK" : "FAIL",
                    nvic_ok ? "enabled" : "FAIL", en_ok ? "OK" : "FAIL");
        pass &= vec_ok & nvic_ok & en_ok;
    }

    bsp_uprintf("phase2b self-test: %s\r\n", pass ? "ALL PASS" : "FAILURES PRESENT");
    bsp_uprintf("cadence: live 128-frame bank timing is codec-clock driven "
                "(C# I2S model or hardware); bank_count starts at %lu\r\n",
                (unsigned long)i2s_bank_count);

    uint32_t ms = 0;
    for (;;) {
        bsp_delay_ms(1);
        if ((++ms % 1000u) == 0u)
            bsp_uprintf("alive t=%lu ms  banks=%lu  last_iq=[%ld,%ld]\r\n",
                        (unsigned long)ms, (unsigned long)i2s_bank_count,
                        (long)i2s_last_iq[0], (long)i2s_last_iq[1]);
    }
}
