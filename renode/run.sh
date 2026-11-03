#!/bin/sh
# Build-output runner: run a phase firmware in Renode headless, capture USART1.
#
#   ./run.sh                 # run phase2a-analog, 3 s virtual time
#   ./run.sh <phase-dir>     # run firmwares/<phase-dir>/firmware.elf
#   SECONDS_RUN=5 ./run.sh phase2b-sample-transport
#
# Point RENODE at your Renode launcher if it is not on PATH:
#   RENODE=/path/to/renode_1.17.0-portable/renode ./run.sh phase2a-analog
#
# If renode/devices/*.cs exist they are compiled (I2C device models), and if
# renode/<phase-dir>.repl exists it is loaded after the base platform (per-phase
# attached devices). Prints the captured USART1 output and exits.
set -e
cd "$(dirname "$0")"

# Repo root is the parent of this renode/ dir; resolved from the script's path.
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
RENODE=${RENODE:-renode}
PHASE=${1:-phase2a-analog}
FW="$ROOT/firmwares/$PHASE/firmware.elf"
RUNFOR=${SECONDS_RUN:-3}
UARTLOG=$(mktemp /tmp/x6100-usart1.XXXXXX.log)

if [ ! -f "$FW" ]; then
    echo "firmware not found: $FW (build it with firmwares/$PHASE/build.sh)" >&2
    exit 1
fi

SCRIPT=$(mktemp /tmp/x6100-run.XXXXXX.resc)
{
    # Compile any device models before the platform references their types.
    for cs in "$ROOT"/renode/devices/*.cs; do
        [ -f "$cs" ] && echo "include @$cs"
    done
    echo 'mach create "x6100"'
    echo "machine LoadPlatformDescription @$ROOT/renode/stm32f427.repl"
    [ -f "$ROOT/renode/$PHASE.repl" ] && \
        echo "machine LoadPlatformDescription @$ROOT/renode/$PHASE.repl"
    echo "sysbus LoadELF @$FW"
    echo "cpu VectorTableOffset 0x08020000"
    echo "sysbus.usart1 CreateFileBackend @$UARTLOG true"
    echo "emulation RunFor \"$RUNFOR\""
    echo "quit"
} > "$SCRIPT"

"$RENODE" --console --disable-xwt -e "include @$SCRIPT" >/dev/null 2>&1 || true
rm -f "$SCRIPT"

echo "=== USART1 ($PHASE, ${RUNFOR}s virtual) ==="
cat "$UARTLOG" 2>/dev/null || echo "(no USART1 output captured)"
rm -f "$UARTLOG"
