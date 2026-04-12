# M4 FreeRTOS Firmware — GPIO Toggle Demo

FreeRTOS application for the **Cortex-M4F Core 0** on the **Apalis iMX8QM**
(Ixora V1.2 carrier board). Toggles two LSIO_GPIO2 pins at ~1 Hz.

| Signal        | SoC pad       | Linux equiv.           | Ixora X27 pin |
|---------------|---------------|------------------------|---------------|
| GPIO Pin A    | SC_P_ESAI0_FSR | LSIO_GPIO2[8] (line 8) | 13            |
| GPIO Pin B    | SC_P_ESAI0_FST | LSIO_GPIO2[9] (line 9) | 14            |

## Prerequisites

1. **MCUXpresso SDK** — download from [NXP](https://mcuxpresso.nxp.com/) for
   processor **MIMX8QM6xxxFF** (free NXP account required).

2. **Extract SDK files** into this project:
   ```bash
   ./scripts/extract_sdk.sh /path/to/SDK_2.x.x_MIMX8QM6xxxFF
   ```

3. **FreeRTOS kernel** — the extraction script copies it from the SDK. If not
   found, clone manually:
   ```bash
   git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git freertos/Source
   ```

## Build

### Docker (recommended — used by `docker compose`)

```bash
# From the repo root:
docker compose build m4loader
```

### Local (requires arm-none-eabi-gcc)

```bash
cmake -S . -B build \
    -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Release
cmake --build build
# Outputs: build/gpio_toggle (ELF) and build/gpio_toggle.bin
```

## Deploy

### Option A: RemoteProc (from Linux)

Requires the HMP device tree overlay for Apalis iMX8 to be enabled.

```bash
docker compose up m4loader
```

The loader container will attempt to load `gpio_toggle.elf` via
`/sys/class/remoteproc/remoteproc0`.

### Option B: U-Boot

Copy the `.bin` to the board and configure U-Boot:

```
# In U-Boot console:
setenv load_cmd "ext4load mmc 0:1"
setenv m4_0_image "/ostree/deploy/torizon/var/m4fw/gpio_toggle.bin"
setenv loadm4image_0 '${load_cmd} ${loadaddr} ${m4_0_image}'
saveenv
run m4boot_0
```

To auto-start on every boot:

```
setenv bootcmd "run m4boot_0; ${bootcmd}"
saveenv
reset
```

## Pin Ownership

Once the M4 firmware owns GPIO2 lines 8 and 9, the Linux side (cppsvc)
**must not** attempt to use those lines. The `cppsvc` code has been updated
to only use motor 2 (lines 12/13).

If GPIO toggling does not work, verify:
1. The SCFW resource partition grants `SC_R_GPIO_2` to M4 core 0.
2. The pads are muxed to their LSIO_GPIO alternate function.
3. The Linux device tree does **not** claim these pads.
