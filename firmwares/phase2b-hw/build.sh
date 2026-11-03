#!/bin/sh
# Build x6100-bbfw phase2b-hw (Phase 2a bring-up + live I2S transport) for HARDWARE.
set -e
cd "$(dirname "$0")"
. ../../common/build_common.sh
build_firmware firmware.c "$COMMON_DIR/i2s.c"   # i2s.{c,h} promoted to common/
