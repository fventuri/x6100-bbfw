#!/bin/sh
# Build x6100-bbfw phase3-rx (receive-only radio: LO + RF switching + AM DSP)
# for HARDWARE. Receive-only; PA + charger stay disabled throughout.
set -e
cd "$(dirname "$0")"
. ../../common/build_common.sh
# CMSIS-DSP polyphase FIRs (arm_fir_decimate/_interpolate/_fir f32) are compiled
# straight into the firmware from the cube's DSP sources -- header-consistent and
# no prebuilt-lib/ABI mismatch (the README's prebuilt .a is from an older cube).
# Only the plain-C path of these files compiles on Cortex-M4F (no MVE/NEON).
DSP_INC="$CUBE/Drivers/CMSIS/DSP/Include"
DSP_SRC="$CUBE/Drivers/CMSIS/DSP/Source/FilteringFunctions"
DSP_FILES="$DSP_SRC/arm_fir_decimate_f32.c $DSP_SRC/arm_fir_decimate_init_f32.c \
           $DSP_SRC/arm_fir_interpolate_f32.c $DSP_SRC/arm_fir_interpolate_init_f32.c \
           $DSP_SRC/arm_fir_f32.c $DSP_SRC/arm_fir_init_f32.c"

# -lm after the objects: the hard-float (fpv4-sp) libm multilib is selected by
# the -mfloat-abi=hard flags in build_common.sh. rx_dsp.c's AGC uses log10f/expf.
build_firmware firmware.c rx_dsp.c "$COMMON_DIR/i2s.c" $DSP_FILES -I"$DSP_INC" -lm
