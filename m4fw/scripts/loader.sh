#!/usr/bin/env bash
# =============================================================================
# M4 Firmware Loader — Apalis iMX8QM
#
# Attempts to load the FreeRTOS Gantry firmware onto Cortex-M4 Core 0
# using the Linux remoteproc framework.  If remoteproc is not available
# (e.g. HMP device tree overlay not enabled), falls back to copying the
# .bin file to a persistent location for U-Boot loading.
#
# Usage (inside container):
#   /usr/local/bin/loader.sh
#
# Expected mounts (set in docker-compose.yml):
#   /sys           → host /sys        (for remoteproc sysfs)
#   /host-firmware → host /var/m4fw   (for U-Boot fallback)
# =============================================================================

set -euo pipefail

FW_ELF="/firmware/gantry_fw.elf"
FW_BIN="/firmware/gantry_fw.bin"
FW_NAME="gantry_fw.elf"

RPROC_DIR="/sys/class/remoteproc"
RPROC_DEV=""

# ---------- Helpers ----------

log() { echo "[m4loader] $*"; }

find_rproc() {
    if [ ! -d "$RPROC_DIR" ]; then
        return 1
    fi
    for dev in "$RPROC_DIR"/remoteproc*; do
        if [ -d "$dev" ] && [ -f "$dev/state" ]; then
            RPROC_DEV="$dev"
            return 0
        fi
    done
    return 1
}

stop_rproc() {
    local state
    state=$(cat "$RPROC_DEV/state" 2>/dev/null || echo "unknown")
    if [ "$state" = "running" ]; then
        log "Stopping remote processor ($(basename "$RPROC_DEV"))..."
        echo stop > "$RPROC_DEV/state"
        sleep 1
    fi
}

# ---------- RemoteProc path ----------

load_via_remoteproc() {
    log "RemoteProc device found: $(basename "$RPROC_DEV")"

    stop_rproc

    # remoteproc looks for firmware in /lib/firmware by default.
    # Torizon's rootfs is read-only (OSTree), so redirect to /tmp.
    local fw_dir="/tmp/m4fw"
    mkdir -p "$fw_dir"
    cp "$FW_ELF" "$fw_dir/$FW_NAME"

    # Tell remoteproc where to find firmware
    if [ -f /sys/module/firmware_class/parameters/path ]; then
        echo -n "$fw_dir" > /sys/module/firmware_class/parameters/path
    fi

    # Set firmware name and start
    echo "$FW_NAME" > "$RPROC_DEV/firmware"
    log "Loading firmware: $FW_NAME"
    echo start > "$RPROC_DEV/state"

    local new_state
    new_state=$(cat "$RPROC_DEV/state" 2>/dev/null || echo "unknown")
    log "Remote processor state: $new_state"

    if [ "$new_state" = "running" ]; then
        log "M4 Core 0 is running FreeRTOS gantry firmware."
        log "  Motor 1: LSIO_GPIO2[8:9]   (Ixora X27 pins 13/14)"
        log "  Motor 2: LSIO_GPIO2[12:13]  (Ixora X27 pins 15/16)"
        return 0
    else
        log "WARNING: Remote processor did not start (state=$new_state)."
        log "Falling back to U-Boot path."
        return 1
    fi
}

# ---------- U-Boot fallback path ----------

uboot_fallback() {
    local dest="/host-firmware"

    if [ -d "$dest" ]; then
        cp "$FW_BIN" "$dest/gantry_fw.bin"
        cp "$FW_ELF" "$dest/gantry_fw.elf"
        log "Firmware copied to host at /var/m4fw/"
    else
        log "WARNING: /host-firmware mount not available."
        log "  Copy firmware manually from this container:"
        log "    docker cp <container>:/firmware/gantry_fw.bin /var/m4fw/"
    fi

    log ""
    log "=== U-Boot Configuration Required ==="
    log ""
    log "Connect to the U-Boot console and run:"
    log ""
    log "  setenv load_cmd \"ext4load mmc 0:1\""
    log "  setenv m4_0_image \"/ostree/deploy/torizon/var/m4fw/gantry_fw.bin\""
    log "  setenv loadm4image_0 '\${load_cmd} \${loadaddr} \${m4_0_image}'"
    log "  saveenv"
    log ""
    log "To run the firmware once:"
    log "  run m4boot_0"
    log ""
    log "To auto-start on every boot:"
    log "  setenv bootcmd \"run m4boot_0; \${bootcmd}\""
    log "  saveenv"
    log "  reset"
    log ""
}

# ---------- Main ----------

log "Apalis iMX8QM M4 Firmware Loader"
log "Firmware: gantry_fw (FreeRTOS gantry controller)"

if find_rproc; then
    load_via_remoteproc || uboot_fallback
else
    log "RemoteProc not available (HMP overlay may not be enabled)."
    uboot_fallback
fi

# Keep the container alive briefly so logs can be inspected
log "Loader finished. Container will exit in 10 seconds."
sleep 10
