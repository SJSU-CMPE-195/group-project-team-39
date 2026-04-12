#!/usr/bin/env bash
#
# Extract the minimum required files from the NXP MCUXpresso SDK
# (MIMX8QM6xxxFF) into the m4fw project tree.
#
# Usage:
#   ./extract_sdk.sh /path/to/SDK_2.x.x_MIMX8QM6xxxFF
#
# The SDK must be downloaded from https://mcuxpresso.nxp.com/ (free account).

set -euo pipefail

if [ $# -ne 1 ]; then
    echo "Usage: $0 <path-to-mcuxpresso-sdk>"
    echo ""
    echo "Download the MCUXpresso SDK for MIMX8QM6xxxFF from:"
    echo "  https://mcuxpresso.nxp.com/"
    echo ""
    echo "Then run this script with the extracted SDK directory as argument."
    exit 1
fi

SDK_DIR="$1"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ ! -d "$SDK_DIR/devices/MIMX8QM6" ]; then
    echo "ERROR: $SDK_DIR does not look like a valid MCUXpresso SDK for MIMX8QM6."
    echo "Expected directory: $SDK_DIR/devices/MIMX8QM6/"
    exit 1
fi

echo "=== Extracting MCUXpresso SDK files into $PROJECT_DIR ==="

# --- CMSIS core headers ---
echo "[1/6] CMSIS headers..."
mkdir -p "$PROJECT_DIR/CMSIS/Include"
# Try both possible SDK layouts
if [ -d "$SDK_DIR/CMSIS/Core/Include" ]; then
    cp "$SDK_DIR"/CMSIS/Core/Include/*.h "$PROJECT_DIR/CMSIS/Include/"
elif [ -d "$SDK_DIR/CMSIS/Include" ]; then
    cp "$SDK_DIR"/CMSIS/Include/*.h "$PROJECT_DIR/CMSIS/Include/"
else
    echo "  WARNING: CMSIS headers not found in SDK. Check SDK layout."
fi

# --- Device files (headers, startup, linker script, system init) ---
echo "[2/6] Device files for MIMX8QM6 cm4_core0..."
mkdir -p "$PROJECT_DIR/device/gcc"

DEVICE_SRC="$SDK_DIR/devices/MIMX8QM6"
cp "$DEVICE_SRC/MIMX8QM6_cm4_core0.h"                  "$PROJECT_DIR/device/"
cp "$DEVICE_SRC/MIMX8QM6_cm4_core0_features.h"         "$PROJECT_DIR/device/" 2>/dev/null || true
cp "$DEVICE_SRC/system_MIMX8QM6_cm4_core0.h"           "$PROJECT_DIR/device/"
cp "$DEVICE_SRC/system_MIMX8QM6_cm4_core0.c"           "$PROJECT_DIR/device/"
cp "$DEVICE_SRC/fsl_device_registers.h"                 "$PROJECT_DIR/device/"

# Startup assembly and linker script
GCC_SRC="$DEVICE_SRC/gcc"
if [ -d "$GCC_SRC" ]; then
    cp "$GCC_SRC"/startup_MIMX8QM6_cm4_core0.S         "$PROJECT_DIR/device/gcc/" 2>/dev/null || true
    for ld in "$GCC_SRC"/MIMX8QM6*cm4_core0*ram*.ld "$GCC_SRC"/MIMX8QM6*cm4_core0*.ld; do
        [ -f "$ld" ] && cp "$ld" "$PROJECT_DIR/device/gcc/"
    done
fi

# --- SCFW API (System Controller Firmware IPC headers + RPC clients) ---
echo "[3/6] SCFW API (SCU IPC)..."
SCFW_SRC="$DEVICE_SRC/scfw_api"
if [ -d "$SCFW_SRC" ]; then
    rm -rf "$PROJECT_DIR/device/scfw_api"
    cp -R "$SCFW_SRC" "$PROJECT_DIR/device/scfw_api"
else
    echo "  WARNING: SCFW API not found at $SCFW_SRC"
    echo "  The build will fail without SCU IPC headers."
fi

# --- Drivers ---
echo "[4/6] Peripheral drivers (gpio, common, clock)..."
mkdir -p "$PROJECT_DIR/drivers"

DRV_SRC="$SDK_DIR/devices/MIMX8QM6/drivers"
[ ! -d "$DRV_SRC" ] && DRV_SRC="$SDK_DIR/drivers"

for drv in fsl_gpio fsl_common fsl_clock fsl_igpio; do
    for ext in .h .c _imx.h _imx.c; do
        src="$DRV_SRC/${drv}${ext}"
        [ -f "$src" ] && cp "$src" "$PROJECT_DIR/drivers/"
    done
done

for hdr in fsl_common_arm.h fsl_common_arm.c; do
    src="$DRV_SRC/$hdr"
    [ -f "$src" ] && cp "$src" "$PROJECT_DIR/drivers/"
done

# --- Board reference files ---
echo "[5/6] Board reference files from mekmimx8qm hello_world..."
BOARD_REF="$SDK_DIR/boards/mekmimx8qm/demo_apps/hello_world/cm4_core0"
if [ -d "$BOARD_REF" ]; then
    mkdir -p "$PROJECT_DIR/board/reference"
    cp "$BOARD_REF"/board.c          "$PROJECT_DIR/board/reference/" 2>/dev/null || true
    cp "$BOARD_REF"/board.h          "$PROJECT_DIR/board/reference/" 2>/dev/null || true
    cp "$BOARD_REF"/clock_config.c   "$PROJECT_DIR/board/reference/" 2>/dev/null || true
    cp "$BOARD_REF"/clock_config.h   "$PROJECT_DIR/board/reference/" 2>/dev/null || true
    cp "$BOARD_REF"/pin_mux.c        "$PROJECT_DIR/board/reference/" 2>/dev/null || true
    cp "$BOARD_REF"/pin_mux.h        "$PROJECT_DIR/board/reference/" 2>/dev/null || true
    echo "  (Reference files saved to board/reference/ — adapt for Apalis/Ixora)"
else
    echo "  WARNING: Board reference path not found: $BOARD_REF"
fi

# --- FreeRTOS kernel ---
echo "[6/6] FreeRTOS kernel source..."
FREERTOS_SRC="$SDK_DIR/rtos/freertos/freertos-kernel"
[ ! -d "$FREERTOS_SRC" ] && FREERTOS_SRC="$SDK_DIR/rtos/freertos/freertos_kernel"
[ ! -d "$FREERTOS_SRC" ] && FREERTOS_SRC="$SDK_DIR/rtos/freertos"

if [ -d "$FREERTOS_SRC" ]; then
    mkdir -p "$PROJECT_DIR/freertos/Source/include"
    mkdir -p "$PROJECT_DIR/freertos/Source/portable/GCC/ARM_CM4F"
    mkdir -p "$PROJECT_DIR/freertos/Source/portable/MemMang"

    for f in tasks.c queue.c list.c timers.c event_groups.c stream_buffer.c; do
        src="$FREERTOS_SRC/$f"
        [ ! -f "$src" ] && src="$FREERTOS_SRC/Source/$f"
        [ -f "$src" ] && cp "$src" "$PROJECT_DIR/freertos/Source/"
    done

    INC="$FREERTOS_SRC/include"
    [ ! -d "$INC" ] && INC="$FREERTOS_SRC/Source/include"
    [ -d "$INC" ] && cp "$INC"/*.h "$PROJECT_DIR/freertos/Source/include/"

    PORT="$FREERTOS_SRC/portable/GCC/ARM_CM4F"
    [ ! -d "$PORT" ] && PORT="$FREERTOS_SRC/Source/portable/GCC/ARM_CM4F"
    if [ -d "$PORT" ]; then
        cp "$PORT"/*.c "$PROJECT_DIR/freertos/Source/portable/GCC/ARM_CM4F/"
        cp "$PORT"/*.h "$PROJECT_DIR/freertos/Source/portable/GCC/ARM_CM4F/" 2>/dev/null || true
    fi

    MEMMANG="$FREERTOS_SRC/portable/MemMang"
    [ ! -d "$MEMMANG" ] && MEMMANG="$FREERTOS_SRC/Source/portable/MemMang"
    [ -d "$MEMMANG" ] && cp "$MEMMANG"/heap_4.c "$PROJECT_DIR/freertos/Source/portable/MemMang/"
else
    echo "  WARNING: FreeRTOS kernel not found in SDK."
    echo "  Clone it manually: git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git freertos/Source"
fi

echo ""
echo "=== Done ==="
echo ""
echo "Next steps:"
echo "  1. Build with: docker compose build m4loader"
