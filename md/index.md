# x6100-bbfw

An open-source, from-scratch reimplementation of the **baseband firmware** that
runs on the Xiegu X6100's **STM32F427** MCU (the "BASE" processor). It is built
up phase by phase and validated off-hardware in the [Renode](https://renode.io/)
emulator, so each step can be compiled and run without flashing a physical
radio. Everything published so far is **receive-only**.

1. [Overview](/overview/) — the approach, the phase plan, and the safety baseline
1. [Building &amp; flashing](/building/) — toolchain, STM32CubeF4, Renode, the `.xgf` image
1. Firmware phases:
   1. [Phase 2a — analog, telemetry, and codec init](/phase2a-analog/)
   1. [Phase 2b — I²S sample transport](/phase2b-sample-transport/)
   1. [Phase 2b-HW — codec bring-up + live I²S](/phase2b-hw/)
   1. [Phase 3 — receive-only radio](/phase3-rx/)
1. [Attribution](/attribution/) — the reverse-engineering this builds on

The repository is on GitHub at
[fventuri/x6100-bbfw](https://github.com/fventuri/x6100-bbfw), released under the
[MIT license](https://github.com/fventuri/x6100-bbfw/blob/main/LICENSE). The
authors are not affiliated with Xiegu; flashing custom baseband firmware is at
your own risk.
