# x6100-bbfw

An open-source, from-scratch reimplementation of the **baseband firmware** that
runs on the Xiegu X6100's **STM32F427** MCU (the "BASE" processor). The goal is
a clean-room, MIT-licensed alternative to the stock Xiegu baseband firmware,
built up phase by phase and validated off-hardware in the
[Renode](https://renode.io/) emulator so each step can be compiled and run
without flashing a physical radio.

This repository currently publishes the **Phase 2** (analog, telemetry, codec,
and I2S sample transport), **Phase 3** (receive-only radio), and **Phase 4** (the
Phase 3 receiver driven by the stock X6100 Linux control plane) firmwares.

📖 **Documentation site:** <https://fventuri.github.io/x6100-bbfw/> — overview,
the Phase 3 receive-chain architecture (with the GPIO/I²C/I²S hardware each
block drives), the DSP pipeline, the Phase 4 OEM Linux control plane (I²C3
register window + USART6 IQ/telemetry flow packet), the control-tool reference,
and build/flash notes.

> ⚠️ **Hardware / flashing disclaimer.** This is experimental firmware for a
> specific MCU inside the X6100. Flashing custom baseband firmware is **at your
> own risk** and can brick the radio. There is no warranty (see
> [LICENSE](LICENSE)). The authors are not affiliated with Xiegu.

## Credits

This work stands on prior reverse-engineering of the X6100 hardware:

- **Oleg Belousov (R1CBU)'s** [`x6100-base`](https://github.com/strijar/x6100-base)
  notes recover the MCU's hardware contracts (clock tree, GPIO map, I2C/I2S
  topology, codec and converter wiring, RF/LO switching, PA safety). This
  firmware follows those documented contracts. Oleg's repo hasn't been updated
  in a while; a [fork](https://github.com/fventuri/x6100-base) carries the
  additional findings from this work on its **`WIP-franco`** branch, so you
  don't have to re-derive them.
- **Michael Zabielski (SP5TLS)** suggested using Renode, and wrote an
  [MCU reference document](https://github.com/SP5TLS/x6100-base/blob/master/docs/mcu_reference.md).

## Layout

```
common/                 shared board-support package used by every phase
    bsp.{c,h}           clock (250 MHz), GPIO safe-state, USART1, SysTick
    i2c.{c,h}           I2C1/I2C2 helpers
    i2s.{c,h}           four-stream I2S/DMA sample transport
    startup.c           vector table, reset, SystemInit (FPU/VTOR)
    build_common.sh     shared build settings (sourced by each build.sh)
firmwares/<phase>/      one firmware per bring-up phase
    firmware.c          the phase-specific implementation
    *.c / *.h           phase-local sources (DSP, FIR coeffs, ...)
    link.ld             STM32F427 app image (vectors @0x08020000)
    build.sh            arm-none-eabi build -> .elf / .bin / .xgf
    *.py                host-side control/test tools (phases 3–4)
renode/
    stm32f427.repl      Renode platform (extends Renode's STM32F4 base)
    <phase>.repl        per-phase attached devices (I2C slaves, etc.)
    devices/*.cs        ad-hoc C# device models (compiled at run time)
    run.sh              headless runner: build output -> run -> print USART1
tools/
    band_sweep.py       RF band-switching sweep harness (phase 3, over SSH)
    gen_multirate_coeffs.py  generate the phase-3 RX FIR coefficients (scipy)
```

## Prerequisites

- **ARM GNU toolchain** — `arm-none-eabi-gcc`, `objcopy`, `nm` (bare-metal).
- **ST STM32CubeF4** — for the CMSIS device headers and the header-only STM32
  Low-Level (LL) inline API. Not redistributed here (ST license); download it
  from ST and point the build at it:
  ```sh
  export CUBE=/path/to/STM32CubeF4      # default: ~/st/STM32CubeF4
  ```
  No LL `.c` sources are compiled — every LL call used is `__STATIC_INLINE`, and
  the clock tree is set with direct CMSIS register writes. The Phase 3 receiver
  also compiles a few **CMSIS-DSP** FIR sources (`arm_fir_decimate_f32` and
  friends) directly from the Cube's `Drivers/CMSIS/DSP/Source` tree — no prebuilt
  math library is required.
- **openssl** and **python3** — `build.sh` uses them to produce the encrypted
  `.xgf` flash image (AES-256-ECB). Only needed if you intend to flash hardware.
- **Renode** (optional, for off-hardware runs) — the self-contained 1.17.0
  portable build works. Put its launcher on `PATH` or pass `RENODE=/path/to/renode`.

## Conventions

- **Language:** C, using the STM32 **Low-Level (LL)** inline API; the clock tree
  is programmed with direct CMSIS register writes to match the documented raw
  constants.
- **Target:** 32-bit only, **hard float** (`-mcpu=cortex-m4 -mthumb
  -mfloat-abi=hard -mfpu=fpv4-sp-d16`), newlib-nano.
- **Clock:** 250 MHz PLL tree (SYSCLK/HCLK 250 MHz, PCLK1 62.5 MHz, PCLK2
  125 MHz) from a 25 MHz HSE crystal. A 168 MHz build is selectable with
  `HCLK_MHZ=168 ./build.sh`.
- **Diagnostics:** **USART1, 115200 8-N-1** (PA9/PA10, AF7). On hardware this
  surfaces on `/dev/ttyACM1`; in the emulator it is captured by `renode/run.sh`.

## Build

```sh
# build one phase (writes firmware.elf / firmware.bin / firmware.xgf in its dir)
firmwares/phase2a-analog/build.sh
firmwares/phase3-rx/build.sh

# use a Cube install in a non-default location:
CUBE=/opt/STM32CubeF4 firmwares/phase3-rx/build.sh
```

Build artifacts (`*.elf`, `*.bin`, `*.xgf`, `*.map`) are git-ignored; this repo
ships source only.

## Run in Renode (off-hardware)

```sh
# build a phase, then run it headless and print its USART1 output
firmwares/phase2a-analog/build.sh
renode/run.sh phase2a-analog

# point at a specific Renode if it is not on PATH:
RENODE=~/renode_1.17.0-portable/renode renode/run.sh phase2b-sample-transport
```

Renode's stock STM32F4 models do not reproduce live I2S clocking or the DMA
double-buffer, so sample-cadence and BCLK/WCLK lock are validated on hardware;
the firmware's DSP math and register programming are validated deterministically
in the emulator (inject known I/Q, check the result).

## Flashing hardware

`build.sh` emits `firmware.xgf` — the `.bin` encrypted (AES-256-ECB, no pad)
with the Xiegu IAP-bootloader key, which is what the radio's bootloader expects.
Flashing is done with the X6100's own baseband update mechanism. **Do this at
your own risk**; a bad baseband image can leave the radio unresponsive.

## Phase status

Each firmware directory has its own README with a full description and usage.

| Phase | Firmware | Status |
|---|---|---|
| 2a — analog + telemetry + codec init (ADC/DAC, MAX17205 decode, TLV320 + CS4272) | [`phase2a-analog`](firmwares/phase2a-analog/) | ✅ builds; runs in Renode; scaling/decode/codec-init verified |
| 2b — four I2S DMA streams (slave RX + ext TX, double-buffered, Stream3 ISR) | [`phase2b-sample-transport`](firmwares/phase2b-sample-transport/) | ✅ builds; transport logic verified in Renode; live cadence + BCLK/WCLK on hardware |
| 2b — codec bring-up + live I2S (hardware variant: VersaClock, AVDD-LDO, WS-synced enable) | [`phase2b-hw`](firmwares/phase2b-hw/) | ✅ builds; validated on hardware (I2S locks) |
| 3 — receive-only radio (LO + RF switching + AM/SSB DSP, feed-forward AGC) | [`phase3-rx`](firmwares/phase3-rx/) | ✅ builds; AM + SSB receive + speaker audio proven end-to-end on hardware |
| 4 — Phase 3 RX driven by the OEM Linux control plane (I²C3 register window + reg55 handshake + USART6 IQ/telemetry flow packet) | [`phase4-linux`](firmwares/phase4-linux/) | ✅ builds; OEM-faithful VFO/mode/volume/AGC + live panadapter IQ + battery telemetry validated on hardware |

### Safety baseline (held from the start)

Receive-only. The transmit path stays disabled throughout: PA bias channels
zero; PA enable tuple `{PE8=0, PE9=1, PG11=0}` (disabled); PD4 (`+5T`) low;
charger (PF12) off. These are held and re-asserted as the hardware is brought up.

## License

[MIT](LICENSE) © 2026 Franco Venturi.

ST's STM32CubeF4 (a build prerequisite) is distributed by ST under its own
license and is **not** included in this repository.
