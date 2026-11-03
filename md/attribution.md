# Attribution

This firmware is a clean-room reimplementation, but it stands on prior
reverse-engineering of the X6100 hardware and follows a practical method
demonstrated by others.

- **Oleg Belousov's** [`x6100-base`](https://github.com/strijar/x6100-base)
  notes recover the MCU's hardware contracts — the clock tree, GPIO map, I²C/I²S
  topology, codec and converter wiring, RF/LO switching, and PA safety. This
  firmware follows those documented contracts. Oleg's repository has not been
  updated in a while; a [fork](https://github.com/fventuri/x6100-base) carries
  the additional findings from this work on its **`WIP-franco`** branch, so you
  do not have to re-derive them.

- **Michael Zabielski (SP5TLS)** demonstrated the practical approach used here:
  *"info from Oleg's repo + an STM emulator that actually runs firmware"* — i.e.
  developing against [Renode](https://renode.io/) rather than blind hardware
  flashing.

- **ST STM32CubeF4** provides the CMSIS device headers and the header-only STM32
  Low-Level (LL) API the firmware builds against. It is distributed by ST under
  its own license and is **not** included in this repository.

## License

The x6100-bbfw source is released under the
[MIT license](https://github.com/fventuri/x6100-bbfw/blob/main/LICENSE),
© 2026 Franco Venturi.

The authors are not affiliated with Xiegu. "X6100" and "Xiegu" are referenced
only to identify the hardware this firmware targets.
