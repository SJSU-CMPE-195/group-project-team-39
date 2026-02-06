FROM --platform=linux/amd64 torizon/debian-cross-toolchain-arm64:oldstable-rc AS build
WORKDIR /src

RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake ninja-build \
 && rm -rf /var/lib/apt/lists/*

COPY CMakeLists.txt ./
COPY src/ ./src/

RUN cmake -S . -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_SYSTEM_NAME=Linux \
    -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
    -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
    -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ \
 && cmake --build build

FROM torizon/debian:3-bookworm

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ca-certificates \
 && rm -rf /var/lib/apt/lists/*

COPY --from=build /src/build/cppsvc /usr/local/bin/cppsvc

EXPOSE 8080
ENTRYPOINT ["/usr/local/bin/cppsvc"]
