# =============================================================================
# Multi-stage Dockerfile for the iMX8QM M4 FreeRTOS Gantry Firmware
#
# Stage 1 (build):  Cross-compile the Cortex-M4F C++ firmware using
#                    arm-none-eabi-g++.  Produces .elf and .bin artifacts.
#
# Stage 2 (loader): Lightweight Torizon-based image that loads the
#                    compiled firmware onto the M4 core via remoteproc
#                    (or copies it for U-Boot loading).
# =============================================================================

# ---- Build stage ----
FROM ubuntu:22.04 AS build

RUN apt-get update && apt-get install -y --no-install-recommends \
        cmake \
        make \
        gcc-arm-none-eabi \
        libnewlib-arm-none-eabi \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY . /src

RUN cmake -S /src -B /build \
        -DCMAKE_TOOLCHAIN_FILE=/src/cmake/arm-none-eabi-toolchain.cmake \
        -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /build --parallel

# ---- Loader / runtime stage ----
FROM torizon/debian:3-bookworm

RUN apt-get update && apt-get install -y --no-install-recommends \
        coreutils \
    && rm -rf /var/lib/apt/lists/*

# Firmware artifacts
COPY --from=build /build/gantry_fw      /firmware/gantry_fw.elf
COPY --from=build /build/gantry_fw.bin  /firmware/gantry_fw.bin

# Loader script
COPY scripts/loader.sh /usr/local/bin/loader.sh
RUN chmod +x /usr/local/bin/loader.sh

ENTRYPOINT ["/usr/local/bin/loader.sh"]
