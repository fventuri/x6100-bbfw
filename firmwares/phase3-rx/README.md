# phase3-rx

**Phase 3 — receive-only radio (AM / SSB).**

Builds on the proven Phase 2b-HW bring-up (clocks, I2C, VersaClock, TLV320/CS4272
codec, four-stream I2S transport) and adds the three receive-only pieces from
`docs/{frequency_control,rf_switching,rx_pipeline}.md`:

1. **LO** — retune VersaClock OUT1 to `4 × Ftuned` (the QSD divides by 4),
   computed at runtime from the 5P49V60 register spec (`vc_set_out1_hz()`).
2. **RF path** — primary band index on direct GPIO (PD9/10/11/13) + a secondary
   sub-band index on the 24-bit SPI6 image (PG13 SCK / PG14 MOSI / PG12 latch),
   plus the BAND_V DAC2 code.
3. **DSP** — the receive chain on the I/Q banks ([`rx_dsp.c`](rx_dsp.c)): audio
   out of the I2S2ext pair → TLV320 DAC → headphone/speaker.

The default target is **WDOV 1410 kHz** (a strong local AM broadcast carrier).
AM is demodulated as `sqrt(I²+Q²)`, which is invariant to I/Q swap and sign, so
this first receiver doesn't depend on the still-unverified converter polarity.

## Receive DSP ([`rx_dsp.c`](rx_dsp.c))

Overrides the weak `i2s_dsp()` hook in [`common/i2s.c`](../../common/i2s.c).
Each completed I/Q bank (128 complex samples at 100 kHz):

```
normalize (×2⁻³¹) → decimate/4 → demod → decimate/2
  → DC block (carrier removal) → look-ahead AGC → interpolate/8 → Q31 out
```

- **AM** — `|I+jQ|` envelope (offset-invariant).
- **LSB/USB** — phasing detector: a 101-tap Hilbert transform on Q with I
  delayed by its 50-sample group delay. SSB wants the LO on the dial
  (`LOOFF 0`) so the dial offset maps straight to audio pitch.
- **Feed-forward look-ahead AGC** (log-domain, on by default) holds the output
  near a target dBFS; disable it to fall back to a fixed gain.

FIR and Hilbert coefficients live in [`rx_fir_coeffs.h`](rx_fir_coeffs.h) and
[`hilbert_coeffs.h`](hilbert_coeffs.h).

## Build & flash

```sh
./build.sh            # -> firmware.xgf  (flash THIS to hardware)
CUBE=/path/to/STM32CubeF4 ./build.sh
```

`build.sh` links `-lm` (the hard-float newlib libm) for the AGC's `log10f`/`expf`.
This is a hardware firmware; flashing custom baseband firmware is **at your own
risk** (see the [top-level README](../../README.md#flashing-hardware)).

Diagnostics stream over USART1 (115200 8-N-1, `/dev/ttyACM1`): an "alive" line
with I/Q RMS, carrier level, audio peak, AGC gain, and a DSP CPU gauge.

## Controlling it live — `phase3-rx-ctl.py`

The firmware exposes an **I2C3 slave at 7-bit address `0x72`**. The control
utility runs on the radio's R16 Linux CPU and drives it over `/dev/i2c-0`. Each
command is one I2C write `[opcode, b0, b1, b2, b3]` (payload little-endian); the
firmware echoes every command to USART1.

```sh
./phase3-rx-ctl.py                  # interactive REPL on /dev/i2c-0 @ 0x72
./phase3-rx-ctl.py --bus 0 --addr 0x72
echo "T 1410000" | ./phase3-rx-ctl.py   # one-shot (reads commands from stdin)
```

A bare command with no argument prints the value this session last set
(client-side mirror — there is no firmware read-back). `help` lists everything;
up-arrow recalls history.

### Normal control

| Command | Example | Meaning |
|---|---|---|
| `TUNE <hz>` / `T` | `T 1410000` | tune the receiver (Hz); also sets the RF switch + BAND_V |
| `VOLUME <f>` / `V` | `V 8.0` | audio gain / post-AGC trim (0 = silent) |
| `AGC <tgt atk dcy [hang]>` / `A` | `A -12 70 250` | feed-forward AGC: target dBFS, attack/decay/hang ms. `A 0 0 0` disables it (fixed VOLUME) |
| `LOOFF <hz>` / `O` | `O -100` | signed LO offset (ftuned = station + offset; SSB wants 0) |
| `MODE <m>` / `M` | `M USB` | demod mode AM/LSB/USB (or 1/2/3); auto-sets the LO offset |
| `LINMULT <f>` / `L` | `L 0.05` | HPL/MAIN (R16-capture) level, independent of VOLUME (0 = silent) |

### RF-path explorer / debug

These bypass the frequency→index logic to probe the hardware directly.

| Command | Example | Meaning |
|---|---|---|
| `RFPRIM <0..9>` / `P` | `P 0` | force the primary RF band index (PD9/10/11/13) |
| `RFSEC <0..6>` / `S` | `S 0` | force the secondary sub-band index (SPI6 image bit) |
| `BANDV <code>` / `B` | `B 0x11D` | force the BAND_V DAC2 code (12-bit) |
| `RFIMG <image>` / `I` | `I 0x2000` | write a raw 24-bit SPI6 image (probe the 74HC595) |
| `SC6 <0..0x3F>` / `C` | `C 0x25` | shift a 6-bit word through the PC5/PB0/PB1 RF chain |
| `SC8PG6 <word>` / `G` | `G 0x1A` | shift an 8-bit word through the PC5/PB0/PG6 RF chain |
| `GPIO <PortPin> <mode>` / `X` | `X G8 4` | drive a pin. mode 0=in 1=in+pd 2=in+pu 3=out-lo 4=out-hi 5=OD-lo 6=OD-hi |
| `CODEC <page> <reg> <val>` / `Z` | `Z 1 0x11 0x00` | write a TLV320 register (all base-0) |
| `DUMP` / `D` | `D` | dump CS4272 / VersaClock / RF state to USART1 |

## Band-switching validation — [`../../tools/band_sweep.py`](../../tools/band_sweep.py)

Runs on the dev PC and exercises every RF switching boundary, graded entirely
from firmware telemetry (no listening required). It drives the BASE over SSH
(via `phase3-rx-ctl.py` deployed on the radio) and reads USART1 from a growing
capture log. See that script's header for the SSH/host assumptions.

```sh
../../tools/band_sweep.py --log /tmp/mcu.log            # full sweep
../../tools/band_sweep.py --log /tmp/mcu.log --quick
../../tools/band_sweep.py --log /tmp/mcu.log --only carriers
```

It's silent (never touches VOLUME) and measures pre-AGC, so the AGC can stay on.

## Safety

Receive-only. The PA tuple stays in its RX/disabled state, the charger stays
off, and the I/Q transmit pair (I2S3ext) stays zero-filled. No code path here
asserts the TX tuple.
