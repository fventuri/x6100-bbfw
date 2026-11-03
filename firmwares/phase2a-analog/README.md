# phase2a-analog

**Phase 2a — analog front end, telemetry, and codec/converter init.**

The analog/I2C half of Phase 2. It brings up everything that does *not* depend
on the external I2S clock (that part is Phase 2b). Receive-only: the PA and
charger stay disabled and the codec is left muted, so running it is safe.

## What it does

- **ADC** (`analog.md`): two independent 3-channel scan groups driven by DMA2.
  - ADC1 ch0/1/2 → DMA2 Stream0 ch0 (fast RF measurements).
  - ADC2 ch6/7/14 → DMA2 Stream2 ch1 (15 halfwords = 5 scans; telemetry), with
    the documented IIR filter `y = 0.92·y + 0.016·sum5` and the
    external-voltage scaling (`mV = getter0 × 4.835165`).
- **DAC**: the internal DAC with the DAC2 **BAND_V** staircase path.
- **MAX17205 fuel-gauge telemetry** (I2C `0x36`): reads and decodes Battery
  voltage, RepSOC, Temperature, and AvgCurrent into engineering units
  (`i2c.md` / `charger.md` scale factors).
- **Codec / converter bring-up**:
  - releases the codec reset (PE15) and runs the exact **TLV320AIC3204**
    startup (clock helper + 58-step sequence) at I2C `0x18`;
  - the `0x4A` six-write companion;
  - the **CS4272** probe/init at `0x10` with the `0x4C` fallback branch.

## Build

```sh
./build.sh            # -> firmware.elf / firmware.bin / firmware.xgf
CUBE=/path/to/STM32CubeF4 ./build.sh
```

## Run in the emulator

```sh
../../renode/run.sh phase2a-analog
```

Prints the USART1 diagnostics. What is validated deterministically in Renode
(model-independent): the ADC2 scaling + IIR self-test, the MAX17205
engineering-unit decode, and codec/converter register-write read-back from the
I2C device models. Live ADC acquisition and I2S/BCLK are hardware concerns
(Phase 2b).

## Safety

PA and charger stay disabled; the codec's outputs stay muted. Bringing the
codec out of reset in this state is safe.
