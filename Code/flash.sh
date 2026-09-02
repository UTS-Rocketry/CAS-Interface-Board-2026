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

# The ELF is named after CMAKE_PROJECT_NAME in CMakeLists.txt (currently
# "CAScode"), NOT after the repo directory. Read it from CMakeLists rather than
# hard-coding it, so renaming the project doesn't silently break this script.
PROJECT_NAME="$(sed -n 's/^[[:space:]]*set(CMAKE_PROJECT_NAME[[:space:]]\+\([A-Za-z0-9_-]\+\).*/\1/p' CMakeLists.txt | head -n1)"
if [ -z "${PROJECT_NAME}" ]; then
    echo "!!! Could not determine CMAKE_PROJECT_NAME from CMakeLists.txt" >&2
    exit 1
fi
ELF="${BUILD_DIR}/${PROJECT_NAME}.elf"

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
    echo "    ELF files actually present in ${BUILD_DIR}:" >&2
    find "${BUILD_DIR}" -maxdepth 1 -name '*.elf' -printf '      %f\n' 2>/dev/null || echo "      (none)" >&2
    exit 1
fi

# ---- build-configuration report ------------------------------------------
# These flags change what actually flies. Printing them every build makes it
# much harder to fly a binary that still has HIL_SIM or a test sequence in it.
echo ">>> Active build flags:"
grep -E '^[[:space:]]*[A-Z_]+[[:space:]]*$' CMakeLists.txt \
    | sed 's/^[[:space:]]*/      /' || true

if grep -qE '^[[:space:]]*HIL_SIM[[:space:]]*$' CMakeLists.txt; then
    echo "    *** WARNING: HIL_SIM is enabled - sensors are SIMULATED, not real. ***"
fi
if grep -qE '^[[:space:]]*DEBUG[[:space:]]*$' CMakeLists.txt; then
    echo "    *** WARNING: DEBUG is enabled - blocking printf in the control loop. ***"
fi
if grep -qE '^[[:space:]]*(SERVO_TEST|SERVO_CHARACTERIZE|AIRBRAKE_TEST_SEQUENCE)[[:space:]]*$' CMakeLists.txt; then
    echo "    *** WARNING: a servo/airbrake TEST MODE is enabled. ***"
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