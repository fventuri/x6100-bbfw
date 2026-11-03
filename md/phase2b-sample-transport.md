# Phase 2b — I²S sample transport

`phase2b-sample-transport` is the full-duplex I²S slave sample transport — the
emulator-oriented build that configures the transport and validates its logic
deterministically. For the on-hardware version that runs it live after a real
codec bring-up, see [Phase 2b-HW](/phase2b-hw/).

It is DSP-less: receive banks are reconstructed to signed 32-bit, transmit banks
are zero-filled so both TX I²S pairs stay electrically inactive.

## What it does

Configures both full-duplex I²S slave pairs and their four DMA1 double-buffers,
installs the `DMA1_Stream3` handler, and goes live:

- AF pins: PA15/PC10/PC12/PB14 = AF6; PC11/PB12/PB13/PB15 = AF5.
- SPI2/I²S2 + SPI3/I²S3 as **I²S slave receive**; I²S2ext/I²S3ext as **slave
  transmit**. Philips, 32-bit data in a 32-bit channel, no MCU MCLK.
- Four DMA1 streams (16/16-bit, MINC, circular, double-buffer, 512 halfwords /
  bank): S0 SPI3-RX, S3 SPI2-RX (+TCIE), S4 I²S2ext-TX, S5 I²S3ext-TX.
- `DMA1_Stream3_IRQHandler`: select completed banks from the CT bits,
  reconstruct signed-32 per channel (`hw[0]<<16 | hw[1]`), zero-fill TX.

## What the emulator can and cannot show

Renode's stock STM32F4 models SPI2/SPI3 as plain SPI (no external-clock I²S
framing), has no I²S2ext/I²S3ext, and its DMA model drops CIRC/DBM/CHSEL and the
double-buffer pointer. So a **live 128-frame cadence and BCLK/WCLK lock cannot be
reproduced** with the stock platform. What *is* validated here: the DMA register
programming, the I²S format programming, signed-32 reconstruction + channel
ordering, the Stream3 handler's bank-ownership selection + TX zero-fill, and the
vector/NVIC wiring. The self-test prints `ALL PASS`.

Live cadence / overrun-freedom is proven on the bench by
[Phase 2b-HW](/phase2b-hw/).

## Build &amp; run

```sh
firmwares/phase2b-sample-transport/build.sh
renode/run.sh phase2b-sample-transport
```
