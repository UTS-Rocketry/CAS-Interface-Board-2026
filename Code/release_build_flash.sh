#!/bin/bash
set -e

# ============================================================
# FLIGHT (RELEASE) BUILD + FLASH
# Usage:
#   ./release_build_flash.sh                  # clean flight build
#   ./release_build_flash.sh --airbrake-test   # include airbrake test sequence
# ============================================================

AIRBRAKE_TEST=0
for arg in "$@"; do
  case "$arg" in
    --airbrake-test)
      AIRBRAKE_TEST=1
      ;;
    *)
      echo "Unknown argument: $arg"
      exit 1
      ;;
  esac
done

EXTRA_DEFINES=""
if [ "$AIRBRAKE_TEST" -eq 1 ]; then
  echo ""
  echo "!!! Building WITH AIRBRAKE_TEST_SEQUENCE enabled !!!"
  echo "!!! Confirm this is intentional for this flight  !!!"
  echo ""
  read -p "Type 'yes' to confirm: " confirm
  if [ "$confirm" != "yes" ]; then
    echo "Aborted."
    exit 1
  fi
  EXTRA_DEFINES="-DAIRBRAKE_TEST_SEQUENCE"
fi

echo "=== FLIGHT (RELEASE) BUILD ==="
echo "Reconfiguring as Release (DEBUG off)..."
rm -rf build
cmake -B build \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_FLAGS="$EXTRA_DEFINES"

echo "Building..."
cmake --build build

# --- Safety check: confirm no test/debug/sim flags leaked into the build ---
UNSAFE_FLAGS="DDEBUG|DHIL_SIM"
if [ "$AIRBRAKE_TEST" -eq 0 ]; then
  UNSAFE_FLAGS="$UNSAFE_FLAGS|DAIRBRAKE_TEST_SEQUENCE"
fi

if grep -qE "$UNSAFE_FLAGS" build/compile_commands.json; then
  echo ""
  echo "!!! WARNING: unsafe compile flag(s) found — this is NOT a clean release build !!!"
  echo "!!! Check CMakeLists.txt / defines. Aborting flash.                          !!!"
  exit 1
fi
echo "Confirmed: build is clean."

echo "Converting to binary..."
arm-none-eabi-objcopy -O binary build/CAScode.elf build/CAScode.bin

echo "Flashing..."
if command -v st-flash >/dev/null 2>&1 && st-flash write build/CAScode.bin 0x08000000; then
  echo "Flashed via st-flash."
else
  echo "st-flash unavailable or failed — falling back to OpenOCD..."
  openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "program build/CAScode.elf verify reset exit"
fi

echo "=== RELEASE FLASHED ==="