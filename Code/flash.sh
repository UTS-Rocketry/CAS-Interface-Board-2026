#!/usr/bin/env bash
#
# Build and flash the Kestrel control board (STM32F405RG).
# Usage:
#   ./flash.sh           # configure, build, and flash
#   ./flash.sh build     # build only, no flash
#   ./flash.sh clean     # wipe build dir, then configure + build + flash
#
set -euo pipefail

PRESET="Debug"
BUILD_DIR="build/Debug"
ELF="${BUILD_DIR}/Code.elf"

# ---- parse arg ----
MODE="${1:-all}"

if [ "$MODE" = "clean" ]; then
    echo ">>> Wiping build directory"
    rm -rf build
fi

# ---- configure (re-runs CMake; picks up CMakeLists changes like SERVO_TEST) ----
echo ">>> Configuring (preset: ${PRESET})"
cmake --preset "${PRESET}"

# ---- build ----
echo ">>> Building"
cmake --build "${BUILD_DIR}"

if [ ! -f "${ELF}" ]; then
    echo "!!! Build did not produce ${ELF}" >&2
    exit 1
fi

# ---- size report ----
echo ">>> Binary size:"
arm-none-eabi-size "${ELF}" || true

# ---- stop here if build-only ----
if [ "$MODE" = "build" ]; then
    echo ">>> Build complete (flash skipped)"
    exit 0
fi

# ---- flash via OpenOCD ----
echo ">>> Flashing ${ELF}"
openocd \
    -f interface/stlink.cfg \
    -f target/stm32f4x.cfg \
    -c "adapter speed 10" \
    -c "program ${ELF} verify reset exit"

echo ">>> Done. Board reset and running."