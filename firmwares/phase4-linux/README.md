# phase4-linux

**Phase 4 — the Phase 3 receiver driven by the OEM BASE Linux hardware contract.**

This is the [Phase 3 receive-only radio](../phase3-rx/) (LO + RF switching + AM/SSB
DSP, feed-forward AGC) with its custom 5-byte I²C command slave **replaced by the
stock X6100 BASE 1.1.9 Linux interface**. Where `phase3-rx` is controlled by the
bespoke `phase3-rx-ctl.py` opcodes, `phase4-linux` speaks the exact contract the
Xiegu Linux application (`xradio` / `thread_remote_admin`) uses, so it can be
driven by the OEM UI — or by [OpenX6100](https://github.com/OpenX6100/x6100_gui) —
with no app-side changes.

The RX engine (`rx_dsp.c`, the FIR/Hilbert coefficient headers, the LO/RF
switching) is unchanged from `phase3-rx`; everything new here is the **control
and telemetry plane** that sits in front of it.

## The OEM hardware contract

Two channels, both reverse-engineered in `x6100-base/docs/{protocol,
linux_hardware_interface}.md`:

### 1. I²C3 register window — control (STM32 is the slave @ `0x72`)

The R16 Linux CPU is the I²C master on `/dev/i2c-0`; the STM32 answers at 7-bit
address `0x72`. Unlike the Phase 3 opcode protocol, this is a **byte-addressed
register file** and the read and write windows are *direction-dependent* (the
same address reaches different RAM depending on direction):

- **WRITE** `[off_hi][off_lo][payload…]` (offset big-endian, payload
  little-endian). `off < 0xF000` lands in the **56-word control file** (0xE0
  bytes); `0xF000…0xFFFD` is the calibration window; `0xFFFE…` is the command
  window.
- **READ** is an offset-only 2-byte write (STOP) followed by a separate read;
  the slave keeps the offset across the STOP. `0x0000…0x007F` returns the
  128-byte **firmware-id** string (`"V1.1.9…"`); `0x0100…0x0FFF` returns
  calibration. **You cannot read back a control word you just wrote.**
- **`reg55` boot handshake.** The firmware *tracks* the Linux "boot-ready" bit
  (`reg55 & 0x00100000`) and logs `Linux boot-ready seen … Continuing`, but does
  **not gate** on it — standalone RX with no Linux app still runs.

`control_scan_apply()` maps the OEM control words onto the RX engine each scan:

| Word(s) | OEM meaning | Drives |
|---|---|---|
| `reg1` / `reg7` | VFO A / B frequency (Hz) | `apply_tune()` (LO + RF switch + BAND_V) |
| `reg4` / `reg10` | VFO A / B mode | `rx_mode` (AM/LSB/USB/CW/NFM…) |
| `reg13` bit0 | active VFO | selects which pair is live |
| `reg14` | `rxvol` (0=mute, 30=0 dB, 59=+29 dB) | `rx_audio_gain` (`powf` transfer) |
| `reg5` / `reg11` | AGC mode | `rx_agc_set` |
| `reg27` / `reg28` | AGC hang / time | `rx_agc_set` |
| `reg15` | RF gain | `rx_rf_gain` + panadapter IIR-decimation zoom |

### 2. USART6 flow packet — IQ + telemetry (`/dev/ttyS1`)

USART6 (PC6 TX / PC7 RX, AF8) streams a **`0x1028`-byte (4136 B) packet** to the
R16 every ~35 ms via **DMA2 Stream 6**, stamped by the STM32 **hardware CRC32**
engine, at **1.5 Mbaud**. DMA2 S6 + USART6 + CRC are otherwise idle (the RX
engine lives on DMA2 S0/S2 + DMA1), so there is no collision.

```
0x0000  magic 0xAA5555AA
0x0004  live 100 kHz raw RX I/Q         (512 cplx fp32, or 1024 cplx bf16)
0x1004  flags   bit3 = ext supply present,  bit4 = charging
0x100C  external-supply voltage          (tenths of a volt, ADC2/PA6)
0x100D  battery voltage                  (tenths of a volt, MAX17205 Batt)
0x100E  battery capacity                 (%, 2S Li-ion discharge-curve LUT)
0x1014  flow_info_t  (rev>=8): lo_freq center, flow_fmt, seq, fft_dec …
0x1024  CRC32
```

- **Live panadapter I/Q.** The DSP ISR snapshots the live 100 kHz raw I/Q into
  the packet; the GUI maps it across the full 100 kHz span for the waterfall.
  Two formats are offered via `flow_info`: `fp32` (512 complex) or **`bf16`
  (1024 complex)** for 2× waterfall resolution at the same packet size. `reg15`
  selects an IIR-decimation **zoom** (`fft_dec`) for a narrower span.
- **Battery / power telemetry** feeds the GUI's power widget (`clock.c`): the
  battery icon + voltage always, the capacity % when on battery, the external
  voltage when plugged in, and a charging bolt when `flag.charging`. `vbat` is
  the MAX17205 cell-stack voltage; `batcap` comes from a **2S Li-ion
  discharge-voltage LUT** (ported from Georgy's `x6100_base_patch`
  `bat_cap_by_voltage`), which is robust even on an imperfectly-provisioned
  gauge — not the MAX17205 `RepSOC`. `charging` requires external power present
  **and** current flowing into the pack (MAX17205 `AvgCurrent` sign). The
  MAX17205 is polled ~1 Hz from the main loop over I²C1.

`flow_info_t` is byte-identical to X6100Control's `flow.h` and Georgy's
`x6100_base_patch` `comm.h`, so the OpenX6100 GUI decodes our bitfields exactly
(it reads the fields only when the BASE reports rev ≥ 8, which our fw-id's `,r8`
suffix advertises).

## Build & flash

```sh
./build.sh                         # -> firmware.xgf  (flash THIS to hardware)
CUBE=/path/to/STM32CubeF4 ./build.sh
```

Same build as `phase3-rx`: the CMSIS-DSP polyphase FIRs are compiled straight
from the Cube's `Drivers/CMSIS/DSP/Source` tree (no prebuilt math library) and
`-lm` is linked for the AGC's `log10f`/`expf` and the `rxvol` `powf` transfer.
This is a hardware firmware; flashing custom baseband firmware is **at your own
risk** (see the [top-level README](../../README.md#flashing-hardware)).

Diagnostics stream over **USART1** (115200 8-N-1, `/dev/ttyACM1`) — *not* part of
the Linux contract: an "alive" line (I/Q RMS, carrier, audio peak, AGC gain, CPU
gauge, and a BMS line), plus every control-register change and the `reg55`
handshake release. Watch these to confirm control writes, since the OEM window
can't read them back.

## Driving it live — `phase4-linux-ctl.py`

Runs on the R16 Linux CPU and speaks the OEM I²C framing directly (not the Phase
3 opcodes), so it's a faithful stand-in for the stock app while bringing the BASE
up by hand.

```sh
./phase4-linux-ctl.py                 # REPL on /dev/i2c-0 @ 0x72
./phase4-linux-ctl.py --dry-run       # print the I2C byte stream, touch no device
echo "ID" | ./phase4-linux-ctl.py     # one-shot (reads commands from stdin)
```

| Command | Example | Meaning |
|---|---|---|
| `TUNE <hz> [a\|b]` | `TUNE 7055000 a` | set VFO A/B frequency (reg1/7) |
| `MODE <name> [a\|b]` | `MODE usb a` | set VFO A/B mode (reg4/10): lsb/usb/cw/cwr/am/nfm (+ `_dig`) |
| `VOL <0..59>` | `VOL 30` | rxvol (reg14): 0=mute, 30=0 dB, 59=+29 dB |
| `VFO <a\|b>` | `VFO b` | select the active VFO (reg13 bit0) |
| `ID` | `ID` | read the 128-byte fw-id @offset 0 (expect `"V1.1.9…"`) |
| `REG <idx> <u32>` | `REG 14 30` | write one control word (offset idx×4, value LE) |
| `WRITE <off> <byte…>` | `WRITE 0 …` | raw write: BE offset then bytes (base-0) |
| `BULK [w0..w55]` | `BULK …` | write the whole 56-word window @0 |
| `CMD <byte…>` | `CMD …` | write the command window @0xFFFE |
| `GO` / `HANDSHAKE` | `GO` | reg55 ← 0x00100001 (release the Linux boot handshake) |
| `INIT` | `INIT` | zero the 56-word window, then `GO` (mimics X6100Control) |

## Validating the flow packet — `ttys1_flow_reader.py`

Reads `/dev/ttyS1` on the R16, resyncs on the `0xAA5555AA` magic, verifies the
hardware CRC32, and decodes the telemetry (ext/battery voltages, capacity, flags)
plus the `flow_info` header and the I/Q block stats — a no-listening check that
the flow transport and telemetry are intact.

## Safety

Receive-only, exactly as Phase 3. The PA tuple stays in its RX/disabled state,
the charger stays off, and the I/Q transmit pair (I2S3ext) stays zero-filled. No
code path here asserts the TX tuple. The `reg55` handshake is tracked but never
gated, and no OEM control word reaches a transmit or charger actuator.
