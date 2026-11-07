# Phase 3 — receive-only radio

`phase3-rx` is the first firmware that actually *receives*. It builds on the
[Phase 2b-HW](/phase2b-hw/) bring-up (clocks, I²C, VersaClock, TLV320/CS4272
codec, four-stream I²S transport) and adds the three receive-only pieces:

1. **LO** — retune the VersaClock 5P49V60 synth so the quadrature local
   oscillator lands at `4 × Ftuned` (the QSD divides by four).
2. **RF path** — select the band-pass filter bank and sub-band, and the enables
   that bring the analog front end alive.
3. **DSP** — decimate the 100 kHz I/Q banks, demodulate (AM envelope or SSB
   Hilbert phasing), hold the level with a look-ahead AGC, and send audio to the
   codec DAC.

The default target is **WDOV 1410 kHz** (a strong local AM broadcast carrier).
AM is demodulated as `√(I²+Q²)`, which is invariant to I/Q swap and sign, so this
first receiver doesn't depend on the still-unverified converter polarity. SSB
(LSB/USB) and a feed-forward AGC are also proven on air.

Everything here is **receive-only**: no stage asserts the PA/transmit tuple, the
charger stays off, and the I/Q transmit pair is held silent.

## Receive signal chain

![Receive signal chain — antenna to speaker](/img/rx-chain.png)

**RF front end — band-pass filter banks.** The antenna feeds switched BPF banks.
The single most important control is **PG2**, the RX front-end master enable
(driven high, open-drain + pull-up); with it low the whole front end sits at the
noise floor.

| Function | Hardware |
|---|---|
| RX front-end enable | **PG2** (open-drain + pull-up, driven high) |
| Primary band (BPF bank) | **PD9 / PD10 / PD11 / PD13** (direct GPIO, binary 1/2/4/8) |
| Secondary sub-band | one bit of the 24-bit **SPI6** image → 74HC595 (SCK **PG13**, MOSI **PG14**, latch **PG12**) |
| Band bias staircase | **BAND_V** = internal **DAC2 → PA5** (12-bit, frequency-dependent) |

The **cf88** serial word (bit-banged on **PC5** data / **PB0** clock / **PB1**
latch) is the TX-power attenuator, *not* an RX filter — it is driven to a fixed
benign value and has no effect on receive.

**QSD mixer + LO.** A quadrature sampling detector down-converts RF to baseband
I and Q. Its local oscillator is **OUT1** of the VersaClock 5P49V60 synth,
programmed over **I²C1 (0x6A**, SCL **PB6** / SDA **PB7**, bus enable **PE14**)
to `4 × Ftuned`. The same synth's **OUT0** is the 26 MHz codec master clock.

**I/Q ADC — CS4272** (I²C **0x10**) samples the analog I/Q and streams it as I²S
(slave).

**I²S RX (I/Q).** The I/Q arrives on **SPI3 / I²S3** as a slave receiver, carried
by **DMA1 Stream0** into a double buffer — **128 complex samples per bank at
100 kHz** (WS on **PA15**).

**DSP.** The STM32F427 (250 MHz) processes each bank — see the pipeline below.

**I²S TX (audio).** Demodulated audio goes out **I²S2ext** (slave transmit) via
**DMA1 Stream4** (WS on **PB12**). The left word is the HPL / R16 capture line;
the right word is HPR, the speaker.

**Codec — TLV320AIC3204** (I²C **0x18**, reset on **PE15**) is the **I²S clock
master** (`R0x1B = 0x3C`) — it generates the BCLK/WCLK that clock all four I²S
streams. Headphone drivers are unmuted via page-1 **R0x10 / R0x11**. HPL drives
the R16 audio-capture line; HPR drives an LM386 and the internal speaker.

### Clock / LO summary

| Signal | Source | Role |
|---|---|---|
| SYSCLK / HCLK 250 MHz | STM32 PLL (25 MHz HSE) | MCU + DSP |
| Codec MCLK 26 MHz | VersaClock **OUT0** (I²C1 0x6A) | TLV320 / CS4272 master clock |
| BCLK / WCLK | TLV320AIC3204 (I²S master) | clocks all four I²S streams |
| LO | VersaClock **OUT1** = 4 × Ftuned | QSD quadrature local oscillator |

Because the external codec is the I²S clock master, the four I²S slave streams
only clock on real hardware — the emulator cannot reproduce live I²S cadence.

## DSP pipeline

The receive DSP lives in `rx_dsp.c`, overriding the weak `i2s_dsp()` hook in
`common/i2s.c`. It runs once per completed I/Q bank (128 complex samples at
100 kHz).

![DSP pipeline — I/Q bank to audio](/img/dsp-pipeline.png)

1. **Normalize** the raw Q31 samples (× 2⁻³¹).
2. **Complex DC block + adaptive I/Q imbalance correction** — strips front-end
   DC and balances the two legs. The reported `iq_rms` is measured here.
3. **Decimate / 2 → 50 kHz, then / 2 → 25 kHz** — a two-stage multirate chain
   (`decA`, 13 taps; `decB`, 23 taps) that yields a clean ±8 kHz complex IF.
4. **Demodulate** → 12.5 kHz: **AM** = the envelope `√(I² + Q²)` followed by a
   `/ 2` decimation; **LSB/USB** = a `/ 2` decimation then a phasing detector
   (101-tap Blackman Hilbert on Q, I delayed 50 samples; `USB = I + H(Q)`,
   `LSB = I − H(Q)`).
5. **Audio DC block** — removes the demodulated carrier.
6. **Feed-forward look-ahead AGC** (log-domain, ~4.8 ms look-ahead) holds the
   output near a target dBFS; can be disabled for a fixed gain.
7. **Interpolate / 8** → 100 kHz.
8. **Pack Q31** into the I²S2ext audio bank (left = HPL at `rx_lin_mult`,
   right = HPR at `rx_audio_gain`).

The decimators and interpolator are **CMSIS-DSP polyphase FIRs**
(`arm_fir_decimate_f32` / `arm_fir_interpolate_f32` / `arm_fir_f32`), so the
multiply-accumulates run only at the decimated rate — no per-input-sample
history shift. On any retune or mode change the main loop flushes every
cross-bank history so a change starts with no stale transient. The filter
coefficients are generated by `tools/gen_multirate_coeffs.py` into
`rx_multirate_coeffs.h` (the Hilbert set stays in `hilbert_coeffs.h`). Each bank
must be processed inside **1.28 ms**; the DWT CPU gauge reports about
**35 % (AM)** to **45 % (USB)** of budget at 250 MHz.

## Controlling it live — `phase3-rx-ctl.py`

The firmware is a slave on **I²C3 at 7-bit address 0x72** (SCL **PA8**, SDA
**PC9**). The control utility runs on the radio's R16 Linux CPU and drives it
over `/dev/i2c-0`; each command is one I²C write `[opcode, b0, b1, b2, b3]`
(payload little-endian), echoed to USART1.

```sh
./phase3-rx-ctl.py                  # interactive REPL on /dev/i2c-0 @ 0x72
echo "T 1410000" | ./phase3-rx-ctl.py   # one-shot
```

A bare command with no argument prints the value this session last set; `help`
lists everything.

### Normal control

| Command | Example | Meaning |
|---|---|---|
| `TUNE <hz>` / `T` | `T 1410000` | tune the receiver (Hz); also sets the RF switch + BAND_V |
| `VOLUME <f>` / `V` | `V 8.0` | audio gain / post-AGC trim (0 = silent) |
| `AGC <tgt atk dcy [hang]>` / `A` | `A -12 70 250` | feed-forward AGC: target dBFS, attack/decay/hang ms. `A 0 0 0` disables it |
| `LOOFF <hz>` / `O` | `O -100` | signed LO offset (ftuned = station + offset; SSB wants 0) |
| `MODE <m>` / `M` | `M USB` | demod mode AM/LSB/USB (or 1/2/3); auto-sets the LO offset |
| `LINMULT <f>` / `L` | `L 0.05` | HPL / MAIN (R16-capture) level, independent of VOLUME |

### RF-path explorer / debug

| Command | Example | Meaning |
|---|---|---|
| `RFPRIM <0..9>` / `P` | `P 0` | force the primary RF band index (PD9/10/11/13) |
| `RFSEC <0..6>` / `S` | `S 0` | force the secondary sub-band index (SPI6 image bit) |
| `BANDV <code>` / `B` | `B 0x11D` | force the BAND_V DAC2 code (12-bit) |
| `RFIMG <image>` / `I` | `I 0x2000` | write a raw 24-bit SPI6 image (probe the 74HC595) |
| `SC6 <0..0x3F>` / `C` | `C 0x25` | shift a 6-bit word through the PC5/PB0/PB1 RF chain |
| `SC8PG6 <word>` / `G` | `G 0x1A` | shift an 8-bit word through the PC5/PB0/PG6 RF chain |
| `GPIO <PortPin> <mode>` / `X` | `X G8 4` | drive a pin (0=in, 1=in+pd, 2=in+pu, 3=out-lo, 4=out-hi, 5=OD-lo, 6=OD-hi) |
| `CODEC <page> <reg> <val>` / `Z` | `Z 1 0x11 0x00` | write a TLV320 register (all base-0) |
| `DUMP` / `D` | `D` | dump CS4272 / VersaClock / RF state to USART1 |

### `band_sweep.py`

A band-switching validator that runs on the dev PC and exercises every RF
switching boundary, graded entirely from firmware telemetry — no listening
required. It drives the BASE over SSH (via `phase3-rx-ctl.py` deployed on the
radio) and reads USART1 from a growing capture log, grading boundary flips
against the verified band tables, per-band front-end liveness, and carrier
contrast on known broadcasters.

```sh
band_sweep.py --log /tmp/mcu.log            # full sweep
band_sweep.py --log /tmp/mcu.log --quick
band_sweep.py --log /tmp/mcu.log --only carriers
```

It is silent (never touches VOLUME) and measures pre-AGC, so the AGC can stay
enabled.

## Safety invariants

- **PG2** high = RX front end on; no code path asserts the PA / transmit tuple.
- The I/Q transmit pair (**I²S3ext**, DMA1 Stream5) is zero-filled → inactive.
- The charger is off; the codec's outputs start muted and boot volume is 0
  (silent) with the AGC on, so there is no power-up thump.
