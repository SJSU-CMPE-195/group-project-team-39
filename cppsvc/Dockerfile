ARG TARGETPLATFORM=linux/amd64
FROM --platform=$TARGETPLATFORM torizon/debian-cross-toolchain-arm64:oldstable-rc AS build
WORKDIR /src

RUN dpkg --add-architecture arm64 && \
    apt-get update && apt-get install -y --no-install-recommends \
    cmake ninja-build \
    libgpiod-dev:arm64 \
    && rm -rf /var/lib/apt/lists/*

COPY CMakeLists.txt ./
COPY main.cpp ./
COPY src/ ./src/
COPY include/ ./include/

RUN cmake -S . -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_SYSTEM_NAME=Linux \
    -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
    -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
    -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++

RUN cmake --build build

FROM torizon/debian:3-bookworm

RUN apt-get update && apt-get install -y --no-install-recommends \
    libgpiod2 \
    gpiod \
    && rm -rf /var/lib/apt/lists/*

# Optional but recommended on Torizon
RUN usermod -a -G gpio torizon

COPY --from=build /src/build/cppsvc /usr/local/bin/cppsvc

USER torizon
ENTRYPOINT ["/usr/local/bin/cppsvc"]