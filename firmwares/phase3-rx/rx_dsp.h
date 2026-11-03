/*
 * x6100-bbfw -- Phase 3 receive DSP (AM envelope demodulator).
 * ===========================================================
 * Overrides the weak common/i2s.c i2s_dsp() hook. Each completed I/Q bank
 * (128 complex samples at 100 kHz from SPI3/I2S3, docs/rx_pipeline.md) is:
 *   normalize (x2^-31) -> decimate/4 -> |I+jQ| envelope -> decimate/2
 *   -> DC block (carrier removal) -> interpolate/8 -> Q31 into the I2S2ext
 *   audio bank (-> TLV320 DAC -> headphone/speaker).
 * The I/Q TX bank stays zero-filled (I2S3ext inactive). AM is chosen for first
 * bring-up because sqrt(I^2+Q^2) is invariant to I/Q swap and sign, so it does
 * not depend on the still-unverified converter polarity.
 */
#ifndef X6100_RX_DSP_H
#define X6100_RX_DSP_H

#include <stdint.h>

/* Overall audio output gain (post-interpolator, pre-Q31). Tunable by ear on
 * hardware; starts conservative to avoid clipping a strong local carrier. With
 * the AGC engaged this stays a post-AGC output trim (the analog of the OEM
 * receive_volume): the AGC holds the level, rx_audio_gain sets loudness. */
extern volatile float rx_audio_gain;

/* HPL / MAIN (R16-capture) output level on the LEFT word, independent of the
 * speaker (rx_audio_gain, RIGHT word). 0 = HPL silent. Set live via CMD_LINUX_AUDIO_MULT. */
extern volatile float rx_lin_mult;

/* Flush all cross-bank DSP histories (FIR delay lines, Hilbert/I-delay, complex
 * and audio DC blocks, I/Q-correction estimates, AGC state) before the next
 * processed bank. Call from main-loop context on any retune or mode change so a
 * change starts with no stale transient. */
void rx_dsp_flush(void);

/* Demodulation mode. AM = |I+jQ| envelope (offset-invariant). LSB/USB = phasing
 * detector (101-tap Hilbert on Q, I delayed by its 50-sample group delay). Set
 * from firmware.c on CMD_MODE; default AM. SSB wants the LO on the dial
 * (lo_offset 0) so the dial offset maps straight to audio pitch. */
#define RX_MODE_AM   1
#define RX_MODE_LSB  2
#define RX_MODE_USB  3
extern volatile int rx_mode;

/* Diagnostics for the alive line (updated once per processed bank). */
extern volatile float rx_iq_rms;     /* RMS of the normalized input I/Q    */
extern volatile float rx_env_dc;     /* mean envelope (carrier level)      */
extern volatile float rx_audio_peak; /* peak |audio| after DC block         */
extern volatile uint32_t rx_dsp_cyc_max; /* peak DSP cycles/bank (DWT); CPU gauge */

/* --- Feed-forward look-ahead AGC (log-domain; ported from the phase3-rx bench
 * experiment). Runs at the 12.5 kHz audio rate on the DC-blocked demod audio,
 * before interpolation; holds the output near `target` dBFS. ------------------ */

/* Current state for the alive line (updated in the DSP ISR). */
extern volatile int   rx_agc_on;       /* 0 => fixed rx_audio_gain fallback   */
extern volatile float rx_agc_gain_db;  /* AGC gain currently applied, dB       */

/* Configure the AGC (call from main-loop context -- recompute uses libm). All
 * params zero (target_cdb == 0 && atk_ms == 0 && dcy_ms == 0) disables the AGC
 * and reverts to the fixed rx_audio_gain path.
 *   target_cdb : target output level in centi-dBFS (e.g. -1200 = -12.0 dBFS)
 *   atk_ms     : attack time constant (gain cut when signal rises), ms
 *   dcy_ms     : decay/release time constant (gain rise), ms
 *   hang_ms    : release hang after an attack, ms (0 = none)                  */
void rx_agc_set(int16_t target_cdb, uint16_t atk_ms, uint16_t dcy_ms,
                uint16_t hang_ms);

/* Process n audio samples in place at the 12.5 kHz rate (called by i2s_dsp). */
void rx_agc_process(float *buf, int n);

#endif /* X6100_RX_DSP_H */
