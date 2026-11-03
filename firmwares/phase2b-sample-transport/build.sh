#!/bin/sh
# Build x6100-bbfw phase2b-sample-transport -> firmware.elf / .bin / .xgf
set -e
cd "$(dirname "$0")"
. ../../common/build_common.sh
build_firmware firmware.c i2s.c
