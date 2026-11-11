# Overview

x6100-bbfw is a clean-room reimplementation of the Xiegu X6100's baseband
firmware — the code that runs on the radio's **STM32F427** "BASE" MCU. It is
developed **one bring-up phase at a time** and validated off-hardware in the
[Renode](https://renode.io/) emulator, so each phase can be built and run
without flashing a radio.

## Approach

- **Clean-room, from documented contracts.** The firmware follows the recovered
  hardware contracts (clock tree, GPIO map, I²C/I²S topology, codec and
  converter wiring, RF/LO switching, PA safety) — see [Attribution](/attribution/).
- **Emulator-first.** Logic that does not depend on live external clocks is
  validated deterministically in Renode (register programming, DSP math,
  reconstruction). Anything that needs the external I²S clock — sample cadence,
  BCLK/WCLK lock — is validated on hardware, because the stock STM32F4 models
  cannot reproduce it.
- **C + STM32 Low-Level API.** 32-bit, hard-float, newlib-nano; the clock tree
  is set with direct CMSIS register writes. See [Building &amp; flashing](/building/).

## Phase plan

| Phase | Firmware | Status |
|---|---|---|
| 2a | [`phase2a-analog`](/phase2a-analog/) | ✅ analog front end, telemetry, codec/converter init |
| 2b | [`phase2b-sample-transport`](/phase2b-sample-transport/) | ✅ four I²S DMA streams (transport logic, Renode-verified) |
| 2b | [`phase2b-hw`](/phase2b-hw/) | ✅ codec bring-up + live I²S on hardware |
| 3 | [`phase3-rx`](/phase3-rx/) | ✅ receive-only radio (AM/SSB), proven on air |
| 4 | [`phase4-linux`](/phase4-linux/) | ✅ Phase 3 RX driven by the OEM Linux control plane (I²C3 window + USART6 IQ/telemetry flow), validated on hardware |

Phases 0–1 (MCU foundation and I²C bus discovery) are part of the development
history; the firmwares documented here are Phases 2 through 4. Remaining phases
(guarded transmit, ATU/productization) will each get their own page.

## Safety baseline

Every firmware here is **receive-only** and holds the same invariants:

- No code path asserts the PA / transmit tuple; the PA bias stays at zero.
- The charger stays off.
- The transmit I²S pair is zero-filled, so it is electrically inactive.
- The codec's outputs start muted; boot volume is silent.

These are held and re-asserted from the first phase onward.
