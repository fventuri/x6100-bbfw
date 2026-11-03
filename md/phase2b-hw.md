# Phase 2b-HW — codec bring-up + live I²S

`phase2b-hw` is the combined, *hardware-oriented* Phase 2b build. It runs the
full [Phase 2a](/phase2a-analog/) bring-up (ADC/DAC/MAX17205 + the complete
TLV320/CS4272 codec init that makes the **TLV320 the I²S clock master**,
`R0x1B = 0x3C`) and then starts the Phase 2b I²S/DMA transport.

Only on real hardware is BCLK/WCLK actually present, so this is the build where
the four DMA double-buffers really clock and the `DMA1_Stream3` ISR advances
`bank_count`. Use [Phase 2b — sample transport](/phase2b-sample-transport/) for
the emulator-side logic validation.

## What it does

1. Phase 2a: ADC1/ADC2 scan groups + DMA2, internal DAC/BAND_V, MAX17205
   telemetry decode, and the exact TLV320AIC3204 + `0x4A` + CS4272/`0x4C`
   startup sequences.
2. Phase 2b: brings up the two full-duplex I²S slave pairs and their four DMA1
   double-buffers, installs the Stream3 ISR, and goes live.

It also includes optional Phase-2 bring-up diagnostics (VersaClock VCO-lock
read-back, codec clock-register dump, I²S clock sniff, per-second SR/NDTR/ISR
dump), off by default.

## Build &amp; flash

```sh
firmwares/phase2b-hw/build.sh           # -> firmware.xgf (flash THIS to hardware)
```

This build is meant for a real radio — there is nothing for the emulator to
clock. Flashing custom baseband firmware is **at your own risk** (see
[Building &amp; flashing](/building/)).

## What to expect on the bench

- BCLK/WCLK lock visible on a scope.
- `bank_count` climbing at ~781 Hz (128 frames / 100 kHz = 1.28 ms per bank),
  reported over USART1 (115200 8-N-1, `/dev/ttyACM1`).

Receive-only throughout: PA and charger disabled, TX banks zero-filled, codec
muted.
