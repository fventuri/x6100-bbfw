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

/* Front-end RF-gain scalar (OEM reg15 rfg low byte transfer). Scales the
 * normalized input I/Q before demod + the flow snapshot (whole RX chain). 1.0 =
 * unity. Set from firmware.c on a reg15 change (control_scan_apply). */
extern volatile float rx_rf_gain;

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

/* --- Flow-packet IQ snapshot (USART6 /dev/ttyS1 waterfall) -------------------
 * The DSP ISR fills a 512-sample complex ring with the 100 kHz raw I/Q (the
 * full +-50 kHz front-end passband). The x6100_gui consumer maps these 512
 * samples across FULL_BW_HZ = 100 kHz, so this MUST be the raw 100 kHz rate.
 * Mirrors the OEM double-handshake (FUN_08023c88): the producer fills only
 * while a block is not pending; on completing 512 samples it marks the block
 * ready and freezes, so the main-loop consumer copies a tear-free static block
 * then releases the producer. 512 samples = 4 of our 128-sample banks. */
#define RX_FLOW_IQ_SAMPLES 512u    /* fp32 complex count (0x1000 B / 8)            */
#define RX_FLOW_IQ_MAX     1024u   /* bf16 complex count (0x1000 B / 4) = ring cap */

/* Copy the latest completed 512-complex block (1024 interleaved f32: I,Q,...,
 * little-endian, == _Complex float) into dst (>= 4096 bytes) and release the
 * producer. Returns 1 if a fresh block was copied, 0 if none is ready (dst is
 * left untouched, so the packet keeps its previous block -- OEM behavior).
 * Called from main-loop context by flow_build(). */
int rx_flow_iq_copy(void *dst);

/* Panadapter zoom: decimate the flow IQ by 2^fft_dec (0..3 => span 100 kHz /
 * 1,2,4,8). Set from firmware.c on a reg15 change; the ISR adopts it at the next
 * bank. rx_flow_iq_block_fft_dec() returns the fft_dec the currently-parked IQ
 * block was captured at, so flow_build() reports a value matching the samples. */
void    rx_flow_set_fft_dec(uint8_t fft_dec);
uint8_t rx_flow_iq_block_fft_dec(void);

/* Flow IQ sample format: 0 = fp32 (512 complex), 1 = bf16 (1024 complex, the
 * high 16 bits of each f32 -> 2x waterfall resolution in the same region). Set
 * from firmware.c on a reg18 (flow_fm_emp bit0) change; adopted at the next bank.
 * rx_flow_iq_block_fmt() returns the format the parked block was captured in, so
 * flow_build() reports a flow_fmt matching the samples. */
void    rx_flow_set_fmt(uint8_t fmt);
uint8_t rx_flow_iq_block_fmt(void);

#endif /* X6100_RX_DSP_H */
