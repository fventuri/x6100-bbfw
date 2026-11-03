/*
 * x6100-bbfw -- Phase 3 receive DSP implementation. See rx_dsp.h.
 * Plain-C float FIRs using the recovered OEM coefficient arrays
 * (rx_fir_coeffs.h, from x6100-base analysis). At 250 MHz with the FPU the
 * whole chain is ~tens of microseconds per 1.28 ms bank, so it runs directly
 * in the DMA1_Stream3 ISR like the OEM handler did (docs/dma.md).
 */
#include "rx_dsp.h"
#include "i2s.h"
#include "rx_fir_coeffs.h"
#include "hilbert_coeffs.h"   /* 101-tap Hilbert transformer for LSB/USB phasing demod */
#include <math.h>   /* log10f / expf for the log-domain AGC (libm; see build.sh) */

#define Q31_SCALE   2147483648.0f   /* 2^31  */
#define Q31_INVSCALE 4.656612873e-10f /* 2^-31 */

/* Single-precision square root via the Cortex-M4F hardware instruction (no
 * libm, no errno, ~14 cycles). */
static inline float f_sqrt(float x)
{
    float r;
    __asm__ ("vsqrt.f32 %0, %1" : "=t"(r) : "t"(x));
    return r;
}

/* Default VOLUME = 0 => boots SILENT until set over I2C (VOLUME / 'V'). This
 * keeps the speaker quiet so other tests (e.g. the R16/HPL capture path) can run
 * without audio. Raise by ear once receiving: with the fixed-gain path and the
 * HP drivers at +29 dB (codec_unmute_hp) ~20 is comfortable on WDOV 1410 kHz AM;
 * with the AGC engaged this is a post-AGC trim, so a much smaller value (and/or
 * a lower HP gain) is wanted. Lower if a strong station clips to_q31(). */
volatile float rx_audio_gain = 0.0f;
/* HPL / R16-capture output level (left word), separate from the speaker (right
 * word = rx_audio_gain). 0 = HPL silent. Settable live (CMD_LINUX_AUDIO_MULT).
 * The experiment used ~0.051; default 0 here so the capture path stays quiet
 * until explicitly enabled (the AGC runs the digital path hot). */
volatile float rx_lin_mult = 0.0f;
volatile int   rx_mode = RX_MODE_AM;   /* AM envelope by default */
volatile float rx_iq_rms;
volatile float rx_env_dc;
volatile float rx_audio_peak;

/* Peak DSP cycles seen for one bank (DWT->CYCCNT). The main loop converts it to
 * a worst-case per-bank CPU % and clears it each report period. DWT is enabled
 * in app_main before the I2S DSP starts. Useful headroom gauge as heavier demods
 * (SSB/FM) are added. */
volatile uint32_t rx_dsp_cyc_max;

/* ---- FIR primitives (z[0] = newest sample; histories persist across banks) ----
 * Decimation phase is continuous across banks because 128 % 4 == 0 and
 * 32 % 2 == 0, so an output is emitted at every input index i with
 * i % M == M-1 without needing a saved phase. */
static float dec4_zi[FIR_DEC4_36_TAPS];
static float dec4_zq[FIR_DEC4_36_TAPS];
static float dec2_z[FIR_DEC2_36_TAPS];
static float int8_z[FIR_INT8_64_TAPS / 8u];
/* SSB-only (LSB/USB phasing detector) histories. */
static float dec2_zi[FIR_DEC2_36_TAPS];  /* I decimate-by-2 (25k->12.5k) */
static float dec2_zq[FIR_DEC2_36_TAPS];  /* Q decimate-by-2 (25k->12.5k) */
static float hil_z[HILBERT_TAPS];        /* 101-tap Hilbert FIR, run on Q */
static float idel_z[HILBERT_DELAY + 1];  /* I delay = Hilbert group delay (50) */

/* Complex (pre-decimation) DC block: one-pole HPF per component,
 * y[n] = x[n] - x[n-1] + 0.995*y[n-1] (~80 Hz corner @100 kHz). Removes the
 * static front-end I/Q DC offset that otherwise beats the carrier. */
static float dcb_xi1, dcb_yi1, dcb_xq1, dcb_yq1;
/* Adaptive I/Q imbalance correction: leaky energy/cross estimates (decay 0.997)
 * -> amplitude + cross-talk coefficients. Improves SSB opposite-sideband
 * rejection and AM envelope purity. */
static float iq_e1, iq_e2, iq_c;
/* Audio DC-block state (file scope so rx_dsp_reset() can flush it on retune). */
static float dc_x1, dc_y1;

static int fir_decimate(const float *h, int taps, int m, float *z,
                        const float *x, int nin, float *y)
{
    int nout = 0;
    for (int i = 0; i < nin; i++) {
        for (int k = taps - 1; k > 0; k--)
            z[k] = z[k - 1];
        z[0] = x[i];
        if ((i % m) == (m - 1)) {
            float acc = 0.0f;
            for (int k = 0; k < taps; k++)
                acc += h[k] * z[k];
            y[nout++] = acc;
        }
    }
    return nout;
}

/* Polyphase interpolator by L: y[i*L+p] = sum_k h[p + k*L] * x[i-k]. The
 * L factor is folded into the caller's output gain. */
static void fir_interpolate(const float *h, int taps, int l, float *z,
                            const float *x, int nin, float *y)
{
    int phases = taps / l;        /* 64/8 = 8 taps per output phase */
    int o = 0;
    for (int i = 0; i < nin; i++) {
        for (int k = phases - 1; k > 0; k--)
            z[k] = z[k - 1];
        z[0] = x[i];
        for (int p = 0; p < l; p++) {
            float acc = 0.0f;
            for (int k = 0; k < phases; k++)
                acc += h[p + k * l] * z[k];
            y[o++] = acc;
        }
    }
}

static inline int32_t to_q31(float s)
{
    if (s >=  0.9999695f) s =  0.9999695f;   /* clamp just inside full scale */
    if (s <= -0.9999695f) s = -0.9999695f;
    return (int32_t)(s * Q31_SCALE);
}

/* Store a signed-32 sample as two halfwords, high half first -- the inverse of
 * i2s_reconstruct_frame()'s (hw[0]<<16 | hw[1]) read (docs/rx_pipeline.md:
 * "exchange the two 16-bit halves"). */
static inline void pack_word(uint16_t *hw2, int32_t v)
{
    hw2[0] = (uint16_t)((uint32_t)v >> 16);
    hw2[1] = (uint16_t)((uint32_t)v & 0xFFFFu);
}

/* ---- Feed-forward look-ahead AGC (log-domain; Oleg rx_pipeline.md "AGC") ----
 * Ported verbatim from the phase3-rx bench experiment. Runs at the 12.5 kHz
 * audio rate on the DC-blocked demod audio. gain is tracked in dB (constant-dB
 * attack/release like the OEM 20*log10 path); a look-ahead delay lets the gain
 * pull down before a transient reaches the output. Disabled (all params 0) ->
 * the fixed rx_audio_gain path is used instead.
 *
 * A fixed RX_AGC_PREGAIN is applied to the audio before the AGC so the signal
 * sits at the same operating point the experiment was bench-tuned at: this is
 * what lets the experiment's control settings (target/atk/dcy/hang) transfer
 * directly. The AGC normalizes toward `target` regardless of input scale, so
 * the pre-gain only sets where the signal lands within the clamp window. */
#define RX_AGC_PREGAIN 5.0f         /* matches the experiment's x5 pre-gain      */
#define AGC_FS_HZ    12500.0f
#define AGC_ND       120            /* delay-line length (OEM 120)               */
#define AGC_LOOK     60             /* look-ahead tap (OEM 60 => ~4.8 ms)        */
#define AGC_GMIN_DB  (-40.0f)       /* gain clamp (OEM -100..+10; the x5 pre-gain */
#define AGC_GMAX_DB   (60.0f)       /*  shifts the useful window up)             */
#define AGC_LN10_20   0.11512925f   /* ln(10)/20: 10^(x/20) = expf(x*this)       */

volatile int   rx_agc_on;           /* 0 => fixed rx_audio_gain fallback         */
volatile float rx_agc_gain_db;      /* current gain applied (diagnostics)        */

static volatile float g_agc_target_db;   /* target output level, dBFS            */
static volatile float g_agc_atk_coef;    /* dB-domain one-pole coef, attack      */
static volatile float g_agc_dcy_coef;    /* dB-domain one-pole coef, release     */
static volatile int   g_agc_hang_samps;  /* release hang, samples @12.5k         */

static float g_agc_dly[AGC_ND];          /* ISR-only runtime state               */
static int   g_agc_widx;
static int   g_agc_hangc;

/* Process n audio samples in place: level -> desired gain (to hit target) ->
 * attack/release-smoothed dB gain (clamped) -> applied to the look-ahead-delayed
 * sample. */
void rx_agc_process(float *buf, int n)
{
    float gain_db = rx_agc_gain_db;
    for (int k = 0; k < n; ++k) {
        float s = buf[k];
        g_agc_dly[g_agc_widx] = s;
        float a = s < 0.0f ? -s : s;
        float level_db = 20.0f * log10f(a + 1e-9f);
        float want_db  = g_agc_target_db - level_db;           /* gain to reach target */
        if      (want_db > AGC_GMAX_DB) want_db = AGC_GMAX_DB;
        else if (want_db < AGC_GMIN_DB) want_db = AGC_GMIN_DB;
        if (want_db < gain_db) {                               /* louder -> cut (attack) */
            gain_db += g_agc_atk_coef * (want_db - gain_db);
            g_agc_hangc = g_agc_hang_samps;
        } else {                                               /* quieter -> raise after hang */
            if (g_agc_hangc > 0) g_agc_hangc--;
            else gain_db += g_agc_dcy_coef * (want_db - gain_db);
        }
        float gain = expf(gain_db * AGC_LN10_20);
        int ridx = g_agc_widx - AGC_LOOK; if (ridx < 0) ridx += AGC_ND;
        buf[k] = g_agc_dly[ridx] * gain;
        if (++g_agc_widx >= AGC_ND) g_agc_widx = 0;
    }
    rx_agc_gain_db = gain_db;
}

/* Recompute coefficients from the raw ms params and enable/disable (main-loop
 * context -- uses expf). Per-sample one-pole coef = 1 - exp(-T/tau), T = 1/Fs. */
void rx_agc_set(int16_t target_cdb, uint16_t atk_ms, uint16_t dcy_ms,
                uint16_t hang_ms)
{
    rx_agc_on = !(target_cdb == 0 && atk_ms == 0 && dcy_ms == 0);
    g_agc_target_db  = (float)target_cdb / 100.0f;
    float t_ms = 1000.0f / AGC_FS_HZ;    /* 0.08 ms per sample @ 12.5 kHz */
    g_agc_atk_coef = atk_ms ? (1.0f - expf(-t_ms / (float)atk_ms)) : 1.0f;
    g_agc_dcy_coef = dcy_ms ? (1.0f - expf(-t_ms / (float)dcy_ms)) : 1.0f;
    g_agc_hang_samps = (int)((float)hang_ms * (AGC_FS_HZ / 1000.0f) + 0.5f);
    if (!rx_agc_on) {                    /* reset runtime state so re-enable starts clean */
        for (int k = 0; k < AGC_ND; ++k) g_agc_dly[k] = 0.0f;
        g_agc_widx = 0; g_agc_hangc = 0; rx_agc_gain_db = 0.0f;
    }
}

/* Flush every cross-bank DSP history so a tune/mode change starts fresh with no
 * stale transient. Requested from main-loop context (apply_tune) via a flag the
 * ISR acts on at the next bank, so no cross-context race on the histories. */
static volatile uint8_t dsp_reset_req;

void rx_dsp_flush(void) { dsp_reset_req = 1; }

static void dsp_reset_state(void)
{
    for (int k = 0; k < (int)FIR_DEC4_36_TAPS; k++) { dec4_zi[k] = 0.0f; dec4_zq[k] = 0.0f; }
    for (int k = 0; k < (int)FIR_DEC2_36_TAPS; k++) { dec2_z[k] = 0.0f; dec2_zi[k] = 0.0f; dec2_zq[k] = 0.0f; }
    for (int k = 0; k < (int)(FIR_INT8_64_TAPS / 8u); k++) int8_z[k] = 0.0f;
    for (int k = 0; k < HILBERT_TAPS; k++) hil_z[k] = 0.0f;
    for (int k = 0; k <= HILBERT_DELAY; k++) idel_z[k] = 0.0f;
    dcb_xi1 = dcb_yi1 = dcb_xq1 = dcb_yq1 = 0.0f;
    iq_e1 = iq_e2 = iq_c = 0.0f;
    dc_x1 = dc_y1 = 0.0f;
    for (int k = 0; k < AGC_ND; k++) g_agc_dly[k] = 0.0f;
    g_agc_widx = 0; g_agc_hangc = 0; rx_agc_gain_db = 0.0f;
}

/* AM/SSB demodulator. Strong override of the common/i2s.c weak hook. */
void i2s_dsp(const uint16_t *rx_audio, const uint16_t *rx_iq,
             uint16_t *tx_audio, uint16_t *tx_iq)
{
    (void)rx_audio;   /* SPI2/I2S2 RX = TX-mic audio; unused for receive */

    if (dsp_reset_req) { dsp_reset_state(); dsp_reset_req = 0; }  /* flush on retune/mode change */

    uint32_t cyc0 = DWT->CYCCNT;   /* per-bank CPU-cost measurement (DWT enabled in app_main) */

    /* 1. Unpack + normalize 128 complex I/Q samples (ch0 = I, ch1 = Q), then
     * (1b) complex DC block per component (one-pole HPF, a=0.995 ~80 Hz) to
     * strip the static front-end DC offset before it beats the carrier. Energy
     * stats (on the DC-blocked signal) feed the adaptive I/Q correction below. */
    float I[I2S_BANK_FRAMES], Q[I2S_BANK_FRAMES];
    float aE1 = 0.0f, aE2 = 0.0f, aC = 0.0f;
    for (unsigned f = 0; f < I2S_BANK_FRAMES; f++) {
        int32_t ci, cq;
        i2s_reconstruct_frame(&rx_iq[4u * f], &ci, &cq);
        float xi = (float)ci * Q31_INVSCALE;
        float xq = (float)cq * Q31_INVSCALE;
        float i = xi - dcb_xi1 + 0.995f * dcb_yi1;
        float q = xq - dcb_xq1 + 0.995f * dcb_yq1;
        dcb_xi1 = xi; dcb_yi1 = i;
        dcb_xq1 = xq; dcb_yq1 = q;
        I[f] = i; Q[f] = q;
        aE1 += i * i; aE2 += q * q; aC += i * q;
    }
    rx_iq_rms = f_sqrt((aE1 + aE2) / (2.0f * (float)I2S_BANK_FRAMES));

    /* 1c. Adaptive I/Q imbalance correction (phasing-SSB image rejection + AM
     * envelope purity). Update leaky estimates, derive amplitude + cross-talk
     * coefficients (clamped), apply in place: Q' = Q + cross*I, I' = amp*I. */
    iq_e1 = 0.997f * iq_e1 + (0.003f / 128.0f) * aE1;
    iq_e2 = 0.997f * iq_e2 + (0.003f / 128.0f) * aE2;
    iq_c  = 0.997f * iq_c  + (0.003f / 128.0f) * aC;
    float cross = 0.0f, amp = 1.0f;
    if (iq_e1 > 1e-12f) {
        cross = -iq_c / iq_e1;
        amp   = f_sqrt(iq_e2 / iq_e1);
        if (cross >  1.0f) cross =  1.0f; else if (cross < -1.0f) cross = -1.0f;
        if (amp   >  2.0f) amp   =  2.0f; else if (amp   <  0.5f) amp   =  0.5f;
    }
    for (unsigned f = 0; f < I2S_BANK_FRAMES; f++) {
        float i = I[f];
        Q[f] = Q[f] + i * cross;
        I[f] = i * amp;
    }

    /* 2. Decimate complex by 4: 128 -> 32 per component. */
    float Id[I2S_BANK_FRAMES / 4u], Qd[I2S_BANK_FRAMES / 4u];
    fir_decimate(fir_dec4_36, FIR_DEC4_36_TAPS, 4, dec4_zi, I, I2S_BANK_FRAMES, Id);
    fir_decimate(fir_dec4_36, FIR_DEC4_36_TAPS, 4, dec4_zq, Q, I2S_BANK_FRAMES, Qd);

    /* 3+4. Demodulate the 32 complex samples to 16 audio samples (12.5 kHz)
     * per the current mode. rx_env_dc (alive-line signal level) is the mean
     * complex magnitude in both paths. */
    float au[I2S_BANK_FRAMES / 8u];
    if (rx_mode == RX_MODE_AM) {
        /* AM: |I+jQ| envelope @25k, then decimate real by 2 -> 16 @12.5k. */
        float env[I2S_BANK_FRAMES / 4u];
        float envsum = 0.0f;
        for (unsigned k = 0; k < I2S_BANK_FRAMES / 4u; k++) {
            env[k] = f_sqrt(Id[k] * Id[k] + Qd[k] * Qd[k]);
            envsum += env[k];
        }
        rx_env_dc = envsum / (float)(I2S_BANK_FRAMES / 4u);
        fir_decimate(fir_dec2_36, FIR_DEC2_36_TAPS, 2, dec2_z, env,
                     I2S_BANK_FRAMES / 4u, au);
    } else {
        /* LSB/USB phasing detector: decimate I and Q by 2 -> 16 complex @12.5k,
         * H{Q} = 101-tap Hilbert FIR on Q, I delayed by its 50-sample group
         * delay. HW I/Q polarity is inverted vs the textbook detector (bench-
         * verified on WDOV, 2026-10-25), so the signs are flipped:
         *   USB = I_delayed + H{Q};  LSB = I_delayed - H{Q}.
         * With lo_offset 0 the dial offset maps straight to audio. */
        float sbi[I2S_BANK_FRAMES / 8u], sbq[I2S_BANK_FRAMES / 8u];
        fir_decimate(fir_dec2_36, FIR_DEC2_36_TAPS, 2, dec2_zi, Id,
                     I2S_BANK_FRAMES / 4u, sbi);
        fir_decimate(fir_dec2_36, FIR_DEC2_36_TAPS, 2, dec2_zq, Qd,
                     I2S_BANK_FRAMES / 4u, sbq);
        int usb = (rx_mode == RX_MODE_USB);
        float envsum = 0.0f;
        for (unsigned k = 0; k < I2S_BANK_FRAMES / 8u; k++) {
            for (int j = HILBERT_TAPS - 1; j > 0; --j) hil_z[j] = hil_z[j - 1];
            hil_z[0] = sbq[k];
            float hq = 0.0f;
            for (int j = 0; j < HILBERT_TAPS; ++j) hq += hil_z[j] * hilbert_101[j];
            for (int j = HILBERT_DELAY; j > 0; --j) idel_z[j] = idel_z[j - 1];
            idel_z[0] = sbi[k];
            float id = idel_z[HILBERT_DELAY];
            au[k] = usb ? (id + hq) : (id - hq);
            envsum += f_sqrt(sbi[k] * sbi[k] + sbq[k] * sbq[k]);
        }
        rx_env_dc = envsum / (float)(I2S_BANK_FRAMES / 8u);
    }

    /* 5. DC block: strip the carrier level so only the modulation remains.
     * y[n] = x[n] - x[n-1] + 0.9*y[n-1]  (the OEM common-path recurrence;
     * ~200 Hz high-pass at 12.5 kHz). State is file-scope (dc_x1/dc_y1) so
     * rx_dsp_flush() clears it on retune. */
    float peak = 0.0f;
    for (unsigned k = 0; k < I2S_BANK_FRAMES / 8u; k++) {
        float x = au[k];
        float y = x - dc_x1 + 0.9f * dc_y1;
        dc_x1 = x; dc_y1 = y;
        au[k] = y;
        float a = y < 0.0f ? -y : y;
        if (a > peak) peak = a;
    }
    rx_audio_peak = peak;

    /* 5b. AGC: feed-forward look-ahead (log-domain) when enabled, holding the
     * audio near target dBFS. The x5 pre-gain matches the bench operating point
     * so the experiment's control settings transfer directly. When disabled the
     * audio passes through unchanged and loudness is set by rx_audio_gain. */
    if (rx_agc_on) {
        for (unsigned k = 0; k < I2S_BANK_FRAMES / 8u; k++)
            au[k] *= RX_AGC_PREGAIN;
        rx_agc_process(au, (int)(I2S_BANK_FRAMES / 8u));
    }

    /* 6. Interpolate by 8: 16 -> 128 samples back at the 100 kHz output rate. */
    float out[I2S_BANK_FRAMES];
    fir_interpolate(fir_int8_64, FIR_INT8_64_TAPS, 8, int8_z, au,
                    I2S_BANK_FRAMES / 8u, out);

    /* 7. Pack Q31 into the I2S2ext audio bank, dual-channel: LEFT = HPL (MAIN /
     * R16-capture path) at rx_lin_mult, RIGHT = HPR (internal speaker) at
     * rx_audio_gain. The two words carry independent levels so the R16 capture
     * can be fed without driving the speaker, and vice versa. The interpolator's
     * 1/L scaling is undone by these gains. */
    float gl = rx_lin_mult;    /* left  / HPL (R16 capture) */
    float gr = rx_audio_gain;  /* right / HPR (speaker)     */
    for (unsigned f = 0; f < I2S_BANK_FRAMES; f++) {
        pack_word(&tx_audio[4u * f + 0], to_q31(out[f] * gl));   /* left  / HPL */
        pack_word(&tx_audio[4u * f + 2], to_q31(out[f] * gr));   /* right / HPR */
    }

    /* Keep the I/Q transmit pair electrically inactive. */
    for (unsigned i = 0; i < I2S_BANK_HALFWORDS; i++)
        tx_iq[i] = 0u;

    uint32_t cyc = DWT->CYCCNT - cyc0;
    if (cyc > rx_dsp_cyc_max) rx_dsp_cyc_max = cyc;

    i2s_bank_count++;
}
