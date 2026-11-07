/*
 * x6100-bbfw -- Phase 3 receive DSP implementation. See rx_dsp.h.
 *
 * Multirate decimation/interpolation chain (Georgy's plan, 2026-11-06) run
 * through CMSIS-DSP polyphase FIRs (arm_fir_decimate_f32 / _interpolate_f32 /
 * arm_fir_f32 -- MACs at the output/input rate only, no per-sample history
 * shift). Replaces the single-rate recovered-OEM chain (rx_fir_coeffs.h):
 *
 *   100 kHz complex I/Q
 *     -> decA /2 -> 50 kHz   (pass[0,8k] stop[42k,50k], 13 taps, run @100k)
 *     -> decB /2 -> 25 kHz   (pass[0,8k] stop[17k,25k], 23 taps, run @50k)
 *   AM : |I+jQ| envelope @25k -> dec25 /2 (real) -> 12.5 kHz audio
 *   SSB: dec25 /2 on I&Q (complex) -> 12.5 kHz, 101-tap Hilbert phasing
 *   both: DC block -> AGC -> interp /8 -> 100 kHz -> Q31 into the I2S2ext bank
 *
 * The 25 kHz complex IF (+-8 kHz) is Georgy's clean AM/NBFM stream. The audio
 * tail (DC block, AGC, interpolator) stays at 12.5 kHz so the hardware-tuned
 * AGC time constants carry over unchanged. Coefficients: rx_multirate_coeffs.h
 * (generated); all decimators/interpolator are even-symmetric linear-phase, so
 * CMSIS's time-reversed coefficient order is a no-op for them. The Hilbert is
 * anti-symmetric, so it is loaded reversed (= negated) at init to reproduce the
 * hardware-verified USB = I + H{Q} / LSB = I - H{Q} sign convention.
 *
 * At 250 MHz with the FPU the whole chain is tens of microseconds per 1.28 ms
 * bank, so it runs directly in the DMA1_Stream3 ISR like the OEM handler.
 */
#include "rx_dsp.h"
#include "i2s.h"
#include "arm_math.h"            /* CMSIS-DSP polyphase FIRs (built into this FW) */
#include "rx_multirate_coeffs.h" /* rx_decA / rx_decB / rx_dec25 / rx_int8        */
#include "hilbert_coeffs.h"      /* 101-tap Hilbert transformer for LSB/USB phasing */
#include <math.h>   /* log10f / expf for the log-domain AGC (libm; see build.sh) */

#define Q31_SCALE   2147483648.0f   /* 2^31  */
#define Q31_INVSCALE 4.656612873e-10f /* 2^-31 */

/* Sample counts down the decimation chain (per bank). */
#define N0    I2S_BANK_FRAMES        /* 128 @ 100 kHz   */
#define N50   (I2S_BANK_FRAMES / 2u) /*  64 @  50 kHz   */
#define N25   (I2S_BANK_FRAMES / 4u) /*  32 @  25 kHz   */
#define N125  (I2S_BANK_FRAMES / 8u) /*  16 @  12.5 kHz */

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

/* ---- CMSIS-DSP polyphase FIR instances + their persistent state buffers ----
 * Each state buffer is (numTaps + blockSize - 1) floats and holds the cross-bank
 * history; the decimation phase is maintained internally by CMSIS across calls
 * (blockSize is constant and a multiple of M every bank). Flushed on retune by
 * zeroing the state (dsp_reset_state). */
static arm_fir_decimate_instance_f32 decA_i, decA_q;   /* 100->50 kHz complex */
static arm_fir_decimate_instance_f32 decB_i, decB_q;   /*  50->25 kHz complex */
static arm_fir_decimate_instance_f32 dec25_env;        /*  25->12.5 AM env (real) */
static arm_fir_decimate_instance_f32 dec25_i, dec25_q; /*  25->12.5 SSB (complex) */
static arm_fir_instance_f32          hil;              /* 101-tap Hilbert on Q */
static arm_fir_interpolate_instance_f32 int8;          /* 12.5->100 kHz /8    */

static float decA_i_st[RX_DECA_TAPS + N0 - 1u],   decA_q_st[RX_DECA_TAPS + N0 - 1u];
static float decB_i_st[RX_DECB_TAPS + N50 - 1u],  decB_q_st[RX_DECB_TAPS + N50 - 1u];
static float dec25_env_st[RX_DEC25_TAPS + N25 - 1u];
static float dec25_i_st[RX_DEC25_TAPS + N25 - 1u], dec25_q_st[RX_DEC25_TAPS + N25 - 1u];
static float hil_st[HILBERT_TAPS + N125 - 1u];
static float int8_st[(RX_INT8_TAPS / 8u) + N125 - 1u];
static float hil_rev[HILBERT_TAPS];   /* Hilbert coeffs reversed for CMSIS order */
static float idel_z[HILBERT_DELAY + 1]; /* SSB I delay = Hilbert group delay (50) */

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

static void zero_f(float *p, unsigned n) { for (unsigned k = 0; k < n; k++) p[k] = 0.0f; }

/* One-time CMSIS instance setup (ISR context, single-threaded; guarded below). */
static void rx_filters_init(void)
{
    arm_fir_decimate_init_f32(&decA_i, RX_DECA_TAPS, 2, rx_decA, decA_i_st, N0);
    arm_fir_decimate_init_f32(&decA_q, RX_DECA_TAPS, 2, rx_decA, decA_q_st, N0);
    arm_fir_decimate_init_f32(&decB_i, RX_DECB_TAPS, 2, rx_decB, decB_i_st, N50);
    arm_fir_decimate_init_f32(&decB_q, RX_DECB_TAPS, 2, rx_decB, decB_q_st, N50);
    arm_fir_decimate_init_f32(&dec25_env, RX_DEC25_TAPS, 2, rx_dec25, dec25_env_st, N25);
    arm_fir_decimate_init_f32(&dec25_i,   RX_DEC25_TAPS, 2, rx_dec25, dec25_i_st,   N25);
    arm_fir_decimate_init_f32(&dec25_q,   RX_DEC25_TAPS, 2, rx_dec25, dec25_q_st,   N25);
    /* Hilbert is anti-symmetric: reversed(h)=-h; store reversed so arm_fir_f32's
     * time-reversed convention yields the same H{Q} as the old normal-order loop. */
    for (unsigned k = 0; k < HILBERT_TAPS; k++) hil_rev[k] = hilbert_101[HILBERT_TAPS - 1u - k];
    arm_fir_init_f32(&hil, HILBERT_TAPS, hil_rev, hil_st, N125);
    arm_fir_interpolate_init_f32(&int8, 8, RX_INT8_TAPS, rx_int8, int8_st, N125);
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
    zero_f(decA_i_st, RX_DECA_TAPS + N0 - 1u);  zero_f(decA_q_st, RX_DECA_TAPS + N0 - 1u);
    zero_f(decB_i_st, RX_DECB_TAPS + N50 - 1u); zero_f(decB_q_st, RX_DECB_TAPS + N50 - 1u);
    zero_f(dec25_env_st, RX_DEC25_TAPS + N25 - 1u);
    zero_f(dec25_i_st, RX_DEC25_TAPS + N25 - 1u); zero_f(dec25_q_st, RX_DEC25_TAPS + N25 - 1u);
    zero_f(hil_st, HILBERT_TAPS + N125 - 1u);
    zero_f(int8_st, (RX_INT8_TAPS / 8u) + N125 - 1u);
    zero_f(idel_z, HILBERT_DELAY + 1);
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

    static int inited;
    if (!inited) { rx_filters_init(); inited = 1; }              /* one-time CMSIS setup */

    if (dsp_reset_req) { dsp_reset_state(); dsp_reset_req = 0; }  /* flush on retune/mode change */

    uint32_t cyc0 = DWT->CYCCNT;   /* per-bank CPU-cost measurement (DWT enabled in app_main) */

    /* 1. Unpack + normalize 128 complex I/Q samples (ch0 = I, ch1 = Q), then
     * (1b) complex DC block per component (one-pole HPF, a=0.995 ~80 Hz) to
     * strip the static front-end DC offset before it beats the carrier. Energy
     * stats (on the DC-blocked signal) feed the adaptive I/Q correction below. */
    float I[N0], Q[N0];
    float aE1 = 0.0f, aE2 = 0.0f, aC = 0.0f;
    for (unsigned f = 0; f < N0; f++) {
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
    rx_iq_rms = f_sqrt((aE1 + aE2) / (2.0f * (float)N0));

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
    for (unsigned f = 0; f < N0; f++) {
        float i = I[f];
        Q[f] = Q[f] + i * cross;
        I[f] = i * amp;
    }

    /* 2. Multirate complex decimation to 25 kHz: 100->50 (decA) then 50->25
     * (decB), I and Q independently. Id/Qd are 32 complex samples at 25 kHz,
     * band-limited to +-8 kHz (Georgy's clean AM/NBFM IF). Polyphase: MACs at
     * the decimated rate only, no per-input-sample history shift. */
    float I50[N50], Q50[N50], Id[N25], Qd[N25];
    arm_fir_decimate_f32(&decA_i, I,   I50, N0);
    arm_fir_decimate_f32(&decA_q, Q,   Q50, N0);
    arm_fir_decimate_f32(&decB_i, I50, Id,  N50);
    arm_fir_decimate_f32(&decB_q, Q50, Qd,  N50);

    /* 3+4. Demodulate the 32 complex samples to 16 audio samples (12.5 kHz)
     * per the current mode. rx_env_dc (alive-line signal level) is the mean
     * complex magnitude in both paths. */
    float au[N125];
    if (rx_mode == RX_MODE_AM) {
        /* AM: |I+jQ| envelope @25k, then decimate the real envelope by 2 -> 12.5k. */
        float env[N25];
        float envsum = 0.0f;
        for (unsigned k = 0; k < N25; k++) {
            env[k] = f_sqrt(Id[k] * Id[k] + Qd[k] * Qd[k]);
            envsum += env[k];
        }
        rx_env_dc = envsum / (float)N25;
        arm_fir_decimate_f32(&dec25_env, env, au, N25);
    } else {
        /* LSB/USB phasing detector: decimate I and Q by 2 -> 16 complex @12.5k,
         * H{Q} = 101-tap Hilbert FIR on Q (arm_fir_f32, reversed coeffs), I
         * delayed by its 50-sample group delay. HW I/Q polarity is inverted vs
         * the textbook detector (bench-verified on WDOV, 2026-10-25), so:
         *   USB = I_delayed + H{Q};  LSB = I_delayed - H{Q}.
         * With lo_offset 0 the dial offset maps straight to audio. */
        float sbi[N125], sbq[N125], hq[N125];
        arm_fir_decimate_f32(&dec25_i, Id, sbi, N25);
        arm_fir_decimate_f32(&dec25_q, Qd, sbq, N25);
        arm_fir_f32(&hil, sbq, hq, N125);        /* H{Q}, group delay 50 samples */
        int usb = (rx_mode == RX_MODE_USB);
        float envsum = 0.0f;
        for (unsigned k = 0; k < N125; k++) {
            for (int j = HILBERT_DELAY; j > 0; --j) idel_z[j] = idel_z[j - 1];
            idel_z[0] = sbi[k];
            float id = idel_z[HILBERT_DELAY];
            au[k] = usb ? (id + hq[k]) : (id - hq[k]);
            envsum += f_sqrt(sbi[k] * sbi[k] + sbq[k] * sbq[k]);
        }
        rx_env_dc = envsum / (float)N125;
    }

    /* 5. DC block: strip the carrier level so only the modulation remains.
     * y[n] = x[n] - x[n-1] + 0.9*y[n-1]  (the OEM common-path recurrence;
     * ~200 Hz high-pass at 12.5 kHz). State is file-scope (dc_x1/dc_y1) so
     * rx_dsp_flush() clears it on retune. */
    float peak = 0.0f;
    for (unsigned k = 0; k < N125; k++) {
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
        for (unsigned k = 0; k < N125; k++)
            au[k] *= RX_AGC_PREGAIN;
        rx_agc_process(au, (int)N125);
    }

    /* 6. Interpolate by 8: 16 -> 128 samples back at the 100 kHz output rate
     * (arm_fir_interpolate_f32, polyphase; prototype DC gain 1 => output 1/L,
     * matching the old operating point so rx_audio_gain/AGC settings transfer). */
    float out[N0];
    arm_fir_interpolate_f32(&int8, au, out, N125);

    /* 7. Pack Q31 into the I2S2ext audio bank, dual-channel: LEFT = HPL (MAIN /
     * R16-capture path) at rx_lin_mult, RIGHT = HPR (internal speaker) at
     * rx_audio_gain. The two words carry independent levels so the R16 capture
     * can be fed without driving the speaker, and vice versa. The interpolator's
     * 1/L scaling is undone by these gains. */
    float gl = rx_lin_mult;    /* left  / HPL (R16 capture) */
    float gr = rx_audio_gain;  /* right / HPR (speaker)     */
    for (unsigned f = 0; f < N0; f++) {
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
