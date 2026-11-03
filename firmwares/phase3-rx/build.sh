#!/bin/sh
# Build x6100-bbfw phase3-rx (receive-only radio: LO + RF switching + AM DSP)
# for HARDWARE. Receive-only; PA + charger stay disabled throughout.
set -e
cd "$(dirname "$0")"
. ../../common/build_common.sh
# -lm after the objects: the hard-float (fpv4-sp) libm multilib is selected by
# the -mfloat-abi=hard flags in build_common.sh. rx_dsp.c's AGC uses log10f/expf.
build_firmware firmware.c rx_dsp.c "$COMMON_DIR/i2s.c" -lm
