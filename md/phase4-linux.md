# Phase 4 — the OEM Linux control plane

`phase4-linux` takes the proven [Phase 3 receiver](/phase3-rx/) — the LO + RF
switching and the AM/SSB DSP, unchanged — and replaces its bespoke I²C command
protocol with the **stock X6100 BASE 1.1.9 Linux hardware contract**. Where
Phase 3 is driven by the custom five-byte opcodes of `phase3-rx-ctl.py`, Phase 4
speaks the exact interface the Xiegu Linux application (`xradio` /
`thread_remote_admin`) uses, so the same firmware is driven by the **OEM UI** —
or by [OpenX6100](https://github.com/OpenX6100/x6100_gui) — with no changes on
the Linux side.

The receive engine is identical to Phase 3; everything new here is the **control
and telemetry plane** that sits in front of it. Two channels carry it, both
recovered in the `x6100-base` `protocol.md` / `linux_hardware_interface.md`
notes (see [Attribution](/attribution/)):

- an **I²C3 register window** — the R16 Linux CPU writes control, reads status;
- a **USART6 flow packet** — the STM32 streams panadapter I/Q and telemetry up.

Everything is still **receive-only**: no OEM control word reaches a transmit or
charger actuator, the PA tuple stays disabled, and the transmit I²S pair is held
silent.

## I²C3 register window — control

The R16 is the I²C master on `/dev/i2c-0`; the STM32 answers as a slave at 7-bit
address **0x72** (SCL **PA8**, SDA **PC9**). Unlike the Phase 3 opcode protocol,
this is a **byte-addressed register file**, and its read and write windows are
*direction-dependent* — the same address reaches different RAM depending on
whether you read or write:

| Access | Address | Reaches |
|---|---|---|
| **WRITE** | `< 0xF000` | the **56-word control file** (0xE0 bytes) |
| **WRITE** | `0xF000…0xFFFD` | calibration window |
| **WRITE** | `0xFFFE…` | command window |
| **READ** | `0x0000…0x007F` | the 128-byte **firmware-id** string (`"V1.1.9…"`) |
| **READ** | `0x0100…0x0FFF` | calibration data |

A write is one message — `[off_hi][off_lo][payload…]`, offset **big-endian**,
payload **little-endian**. A read is an offset-only two-byte write (STOP)
followed by a separate read; the slave keeps the offset across the STOP. Because
the windows are direction-split, **you cannot read back a control word you just
wrote** — control writes are confirmed on the USART1 diagnostic instead.

**The `reg55` boot handshake.** The firmware *tracks* the Linux "boot-ready" bit
(`reg55 & 0x00100000`) and logs `Linux boot-ready seen … Continuing`, but it
does **not gate** on it — a BASE with no Linux app still receives. `control_scan_apply()`
maps the OEM control words onto the Phase 3 RX engine each scan:

| Word(s) | OEM meaning | Drives |
|---|---|---|
| `reg1` / `reg7` | VFO A / B frequency (Hz) | the LO + RF switch + BAND_V (`apply_tune`) |
| `reg4` / `reg10` | VFO A / B mode | AM / LSB / USB / CW / NFM |
| `reg13` bit0 | active VFO | selects which pair is live |
| `reg14` | `rxvol` (0 = mute, 30 = 0 dB, 59 = +29 dB) | audio gain (`powf` transfer) |
| `reg5` / `reg11` | AGC mode | feed-forward AGC enable/profile |
| `reg27` / `reg28` | AGC hang / time | AGC timing |
| `reg15` | RF gain | RF gain + the panadapter IIR-decimation zoom |

## USART6 flow packet — I/Q + telemetry

USART6 (**PC6** TX / **PC7** RX, AF8) streams a **`0x1028`-byte (4136-byte)
packet** to the R16 every **~35 ms** via **DMA2 Stream 6**, stamped by the STM32
**hardware CRC32** engine, at **1.5 Mbaud**. DMA2 S6 + USART6 + the CRC unit are
otherwise idle — the RX engine lives on DMA2 S0/S2 + DMA1 — so there is no
collision.

| Offset | Field |
|---|---|
| `0x0000` | magic `0xAA5555AA` |
| `0x0004` | live 100 kHz raw RX **I/Q** — 512 complex `fp32`, or 1024 complex `bf16` |
| `0x1004` | flags: bit 3 = external supply present, bit 4 = charging |
| `0x100C` | external-supply voltage (tenths of a volt, ADC2 / PA6) |
| `0x100D` | battery voltage (tenths of a volt, MAX17205 cell stack) |
| `0x100E` | battery capacity (%, 2S Li-ion discharge-curve LUT) |
| `0x1014` | `flow_info` (rev ≥ 8): I/Q center `lo_freq`, format, sequence, `fft_dec` |
| `0x1024` | CRC32 |

**Live panadapter I/Q.** The DSP ISR snapshots the live 100 kHz raw I/Q into the
packet; the GUI maps it across the full 100 kHz span for the waterfall. Two
formats are offered through `flow_info`: `fp32` (512 complex) or **`bf16` (1024
complex)** for twice the waterfall resolution at the same packet size. `reg15`
selects an IIR-decimation **zoom** (`fft_dec`) for a narrower span. The
`flow_info` struct is byte-identical to X6100Control's `flow.h` and Georgy's
`x6100_base_patch` `comm.h`, so the OpenX6100 GUI decodes our fields exactly (it
reads them only when the BASE advertises rev ≥ 8, which our fw-id's `,r8` suffix
does).

**Battery / power telemetry** feeds the GUI power widget: the battery icon and
voltage always, the capacity % when on battery, the external voltage when
plugged in, and a charging bolt when the charging flag is set. `vbat` is the
MAX17205 cell-stack voltage; the capacity comes from a **2S Li-ion
discharge-voltage lookup** (robust even on an imperfectly-provisioned gauge),
not the MAX17205 `RepSOC`. "Charging" requires external power present **and**
current flowing into the pack (MAX17205 `AvgCurrent` sign). The MAX17205 is
polled about once a second over I²C1. This is **display-only telemetry** — the
charger stays hardware-controlled and is never actuated by this firmware.

## Driving it live — `phase4-linux-ctl.py`

Runs on the R16 Linux CPU and speaks the OEM I²C framing directly (not the Phase
3 opcodes), so it is a faithful stand-in for the stock app while bringing the
BASE up by hand. `--dry-run` prints the exact I²C byte stream without touching a
device.

```sh
./phase4-linux-ctl.py                 # REPL on /dev/i2c-0 @ 0x72
echo "ID" | ./phase4-linux-ctl.py     # one-shot: read the firmware-id
```

| Command | Example | Meaning |
|---|---|---|
| `TUNE <hz> [a\|b]` | `TUNE 7055000 a` | set VFO A/B frequency (reg1/7) |
| `MODE <name> [a\|b]` | `MODE usb a` | set VFO A/B mode (reg4/10) |
| `VOL <0..59>` | `VOL 30` | set `rxvol` (reg14) |
| `VFO <a\|b>` | `VFO b` | select the active VFO (reg13) |
| `ID` | `ID` | read the 128-byte firmware-id (`"V1.1.9…"`) |
| `REG <idx> <u32>` | `REG 14 30` | write one control word (offset idx × 4, value LE) |
| `GO` / `HANDSHAKE` | `GO` | release the Linux boot handshake (reg55 ← `0x00100001`) |
| `INIT` | `INIT` | zero the 56-word window, then `GO` (mimics X6100Control) |

Because the control window can't be read back, confirm writes on the **USART1**
diagnostic (115200 8-N-1, `/dev/ttyACM1` — *not* part of the Linux contract): it
prints every register change, the handshake release, and an "alive" line with
I/Q RMS, carrier, audio peak, AGC gain, a CPU gauge, and a battery/power line.

## Validating the flow — `ttys1_flow_reader.py`

Reads `/dev/ttyS1` on the R16, resyncs on the `0xAA5555AA` magic, verifies the
hardware CRC32, and decodes the telemetry (external / battery voltages,
capacity, flags), the `flow_info` header, and the I/Q block stats — a
no-listening check that the flow transport and telemetry are intact.

## Safety invariants

- The `reg55` handshake is **tracked, never gated**, and no OEM control word
  reaches a transmit or charger actuator.
- The PA tuple stays in its RX/disabled state; the charger stays off.
- The I/Q transmit pair (**I²S3ext**) stays zero-filled → electrically inactive.
- Battery and power telemetry are **read-only** (MAX17205 over I²C1); the
  charger remains hardware-controlled.
