# Building &amp; flashing

## Prerequisites

- **ARM GNU toolchain** — `arm-none-eabi-gcc`, `objcopy`, `nm` (bare-metal).
- **ST STM32CubeF4** — for the CMSIS device headers and the header-only STM32
  Low-Level (LL) inline API. It is **not** redistributed with this project (ST
  license); download it from ST and point the build at it:
  ```sh
  export CUBE=/path/to/STM32CubeF4      # default: ~/st/STM32CubeF4
  ```
  No LL `.c` sources are compiled — every LL call used is `__STATIC_INLINE`, and
  the clock tree is set with direct CMSIS register writes.
- **openssl** and **python3** — `build.sh` uses them to produce the encrypted
  `.xgf` flash image. Only needed if you intend to flash hardware.
- **Renode** (optional, for off-hardware runs) — the self-contained 1.17.0
  portable build works. Put its launcher on `PATH` or pass `RENODE=/path/to/renode`.

## Conventions

- **Language:** C, using the STM32 Low-Level (LL) inline API; the clock tree is
  programmed with direct CMSIS register writes to match the documented raw
  constants.
- **Target:** 32-bit only, **hard float** (`-mcpu=cortex-m4 -mthumb
  -mfloat-abi=hard -mfpu=fpv4-sp-d16`), newlib-nano.
- **Clock:** a 250 MHz PLL tree from a 25 MHz HSE crystal (SYSCLK/HCLK 250 MHz,
  PCLK1 62.5 MHz, PCLK2 125 MHz). A 168 MHz build is selectable with
  `HCLK_MHZ=168 ./build.sh`.
- **Diagnostics:** USART1, 115200 8-N-1 (PA9/PA10, AF7) → `/dev/ttyACM1`.

## Build

```sh
# build one phase (writes firmware.elf / .bin / .xgf in its dir)
firmwares/phase2a-analog/build.sh
firmwares/phase3-rx/build.sh

# use a Cube install in a non-default location:
CUBE=/opt/STM32CubeF4 firmwares/phase3-rx/build.sh
```

Build artifacts are git-ignored; the repository ships source only.

## Run in Renode (off-hardware)

```sh
firmwares/phase2a-analog/build.sh
renode/run.sh phase2a-analog                       # prints the USART1 output

RENODE=~/renode_1.17.0-portable/renode \
    renode/run.sh phase2b-sample-transport
```

Renode's stock STM32F4 models do not reproduce live I²S clocking or the DMA
double-buffer, so sample cadence and BCLK/WCLK lock are validated on hardware;
the DSP math and register programming are validated deterministically in the
emulator (inject known I/Q, check the result).

## Flashing hardware

`build.sh` emits `firmware.xgf` — the `.bin` encrypted (AES-256-ECB, no pad)
with the Xiegu IAP-bootloader key, which is what the radio's bootloader expects.
Flashing goes through the X6100's own baseband update mechanism.

> **At your own risk.** Flashing custom baseband firmware can leave the radio
> unresponsive. The authors are not affiliated with Xiegu, and there is no
> warranty (see the [MIT license](https://github.com/fventuri/x6100-bbfw/blob/main/LICENSE)).
