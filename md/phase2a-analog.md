# Phase 2a — analog, telemetry, and codec init

`phase2a-analog` is the analog/I²C half of Phase 2 — everything that does *not*
depend on the external I²S clock (that is [Phase 2b](/phase2b-sample-transport/)).
It is receive-only: the PA and charger stay disabled and the codec is left
muted, so running it is safe.

## What it does

- **ADC** (two independent 3-channel scan groups, DMA2):
  - ADC1 ch0/1/2 → DMA2 Stream0 (fast RF measurements);
  - ADC2 ch6/7/14 → DMA2 Stream2 (telemetry), with the documented IIR filter
    `y = 0.92·y + 0.016·sum5` and external-voltage scaling `mV = getter0 × 4.835165`.
- **DAC** — the internal DAC, including the DAC2 **BAND_V** staircase path.
- **MAX17205 fuel-gauge telemetry** (I²C `0x36`): decodes battery voltage,
  RepSOC, temperature, and average current into engineering units.
- **Codec / converter bring-up**:
  - releases the codec reset (**PE15**) and runs the exact TLV320AIC3204 startup
    (clock helper + 58-step sequence) at I²C `0x18`;
  - the `0x4A` six-write companion;
  - the CS4272 probe/init at `0x10` with the `0x4C` fallback branch.

## Build &amp; run

```sh
firmwares/phase2a-analog/build.sh       # -> firmware.elf / .bin / .xgf
renode/run.sh phase2a-analog            # run headless, print USART1
```

In Renode the following are validated deterministically (model-independent): the
ADC2 scaling + IIR self-test, the MAX17205 engineering-unit decode, and
codec/converter register-write read-back from the I²C device models. Live ADC
acquisition and I²S/BCLK are hardware concerns (Phase 2b).

See [Building &amp; flashing](/building/) for prerequisites.
