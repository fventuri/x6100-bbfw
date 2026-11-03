# phase2b-sample-transport

**Phase 2b — the four I2S DMA streams (sample transport).**

The remaining Phase 2 step: the full-duplex I2S slave sample transport. This is
the emulator-oriented build — it configures the transport and validates its
logic deterministically. For the on-hardware version that runs it live after a
real codec bring-up, see [`../phase2b-hw`](../phase2b-hw).

DSP-less: receive banks are reconstructed to signed 32-bit, transmit banks are
zero-filled so both TX I2S pairs stay electrically inactive.

## What it does

Configures both full-duplex I2S slave pairs and their four DMA1 double-buffers
(`spi_i2s.md` / `dma.md`), installs the `DMA1_Stream3` handler, and goes live:

- AF pins: PA15/PC10/PC12/PB14 = AF6; PC11/PB12/PB13/PB15 = AF5.
- SPI2/I2S2 + SPI3/I2S3 as **I2S slave receive**; I2S2ext/I2S3ext as **slave
  transmit**. Philips, 32-bit data in a 32-bit channel, CKPOL = 0, no MCU MCLK.
- Four DMA1 streams (16/16-bit, MINC, circular, double-buffer, 512 halfwords /
  bank): S0 SPI3-RX, S3 SPI2-RX (+TCIE), S4 I2S2ext-TX, S5 I2S3ext-TX.
- `DMA1_Stream3_IRQHandler`: select completed banks from the CT bits,
  reconstruct signed-32 per channel (`hw[0]<<16 | hw[1]`), zero-fill TX.

## Build & run

```sh
./build.sh
../../renode/run.sh phase2b-sample-transport
```

## What the emulator can and cannot show

Renode's stock STM32F4 models SPI2/SPI3 as plain SPI (no external-clock I2S
framing), has no I2S2ext/I2S3ext, and its DMA model drops CIRC/DBM/CHSEL and the
double-buffer pointer. So a **live 128-frame cadence and BCLK/WCLK lock cannot
be reproduced** with the stock platform. What *is* validated here:

- (A) the four DMA streams' register programming (read-back);
- (B) the I2S peripherals' format programming (read-back, model permitting);
- (C) signed-32-bit reconstruction + channel ordering (unit test);
- (D) the Stream3 handler's bank-ownership selection + TX zero-fill, driven with
  synthetic banks;
- (E) the `DMA1_Stream3` vector + NVIC enable wiring.

The self-test prints `ALL PASS`. Live cadence / overrun-freedom is a hardware
measurement — proven by [`phase2b-hw`](../phase2b-hw) on the bench.

> **Hardware note:** on a real radio the external TLV320AIC3204 is the I2S clock
> master (set by the Phase 2a codec bring-up). Without that, no BCLK/WCLK is
> present and the slaves never clock. This build does not run the codec
> bring-up — use `phase2b-hw` for a live hardware test.

## Safety

PA and charger disabled throughout; TX banks are zero → inactive.
