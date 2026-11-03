#!/bin/sh
# Shared build settings for all x6100-bbfw phase firmwares.
# Source this from a phase build.sh, then call: build_firmware <extra-src...>
#
# STM32F427, 32-bit, hard float. Uses ST's STM32CubeF4 for CMSIS device headers
# and the header-only STM32 Low-Level (LL) inline API. No LL .c sources are
# needed: all LL functions used are __STATIC_INLINE and the clock tree is set
# with direct CMSIS register writes (clocks.md).

CUBE=${CUBE:-$HOME/st/STM32CubeF4}
CMSIS_CORE="$CUBE/Drivers/CMSIS/Include"
CMSIS_DEV="$CUBE/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
LL_INC="$CUBE/Drivers/STM32F4xx_HAL_Driver/Inc"

# Each phase build.sh does `cd "$(dirname "$0")"` before sourcing this, so the
# current dir is firmwares/<phase>/ and common/ is two levels up.
COMMON_DIR=$(CDPATH= cd -- ../../common && pwd)

CC=${CC:-arm-none-eabi-gcc}
OBJCOPY=${OBJCOPY:-arm-none-eabi-objcopy}
NM=${NM:-arm-none-eabi-nm}

# HSE_VALUE must match the board's 25 MHz crystal (clocks.md: VERIFIED).
# HCLK_MHZ selects the core clock (250 default, or 168): `HCLK_MHZ=168 ./build.sh`.
DEFS="-DSTM32F427xx -DUSE_FULL_LL_DRIVER -DHSE_VALUE=25000000 -DHCLK_MHZ=${HCLK_MHZ:-250}"
INCS="-I$CMSIS_CORE -I$CMSIS_DEV -I$LL_INC -I$COMMON_DIR"
CFLAGS="-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os \
        -ffreestanding -fno-common -Wall -Wextra \
        -ffunction-sections -fdata-sections $DEFS $INCS"
LDFLAGS="-T link.ld -Wl,--gc-sections -Wl,-Map,firmware.map \
         --specs=nano.specs --specs=nosys.specs -nostartfiles"

COMMON_SRC="$COMMON_DIR/startup.c $COMMON_DIR/bsp.c $COMMON_DIR/i2c.c"

check_cube()
{
    if [ ! -f "$CMSIS_DEV/stm32f427xx.h" ]; then
        echo "STM32CubeF4 not found under '$CUBE'. Set CUBE=/path/to/STM32CubeF4." >&2
        exit 1
    fi
}

# build_firmware <phase-specific .c files...>
build_firmware()
{
    check_cube
    $CC $CFLAGS $LDFLAGS "$@" $COMMON_SRC -o firmware.elf
    $OBJCOPY -O binary firmware.elf firmware.bin

    echo "built firmware.elf / firmware.bin:"
    ls -l firmware.bin
    echo "first 2 words (little-endian MSP, reset_vector):"
    od -An -tx4 -N8 firmware.bin

    if $NM firmware.elf | grep -q uldivmod; then
        echo "WARNING: __aeabi_uldivmod present (64-bit divide) -- check the code" >&2
    fi

    # Encrypt for the IAP bootloader (AES-256-ECB, no pad, Xiegu key); pad 16 B.
    # Only needed for flashing hardware; the emulator loads the .elf directly.
    XGF_KEY=435158494547552d584945594920202059414e4855414e204d4f554c49414e5a
    python3 - <<'PY'
d = open("firmware.bin", "rb").read()
pad = (-len(d)) % 16
open("firmware.pad.bin", "wb").write(d + b"\xFF" * pad)
PY
    openssl enc -aes-256-ecb -nopad -K "$XGF_KEY" -in firmware.pad.bin -out firmware.xgf
    echo "built firmware.xgf (encrypted, flash THIS to hardware):"
    ls -l firmware.xgf
}
