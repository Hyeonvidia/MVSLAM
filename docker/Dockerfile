# MVSLAM dev image — toolchain only (Ubuntu 24.04, linux/arm64 on Apple Silicon = same arch as Jetson).
# Sources are bind-mounted at /workspace; every library except OpenCV comes from the tag-pinned
# submodules, so the build path is identical inside and outside the container.
# All builds and tests of this project run in this container (never on the host).
#
#   docker compose build                                            # image
#   docker compose run --rm dev docker/scripts/container_build.sh   # configure + build + ctest (docker-release)
#   docker compose run --rm dev docker/scripts/run_dataset.sh configs/euroc_stereo_imu.yaml EuRoC/MH01
FROM ubuntu:24.04
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential gcc-13 g++-13 clang cmake ninja-build ccache git ca-certificates pkg-config \
      libopencv-dev python3 python3-numpy time curl unzip \
    && rm -rf /var/lib/apt/lists/*
# Rerun C++ SDK (viewer adapter, optional at CMake level: MVSLAM_WITH_RERUN). The release zip carries
# prebuilt librerun_c for every platform; its CMake downloads and builds a minimal Apache Arrow 18.0.0.
# Installed once here (/opt/rerun) so project configures need neither network nor an Arrow build.
ARG RERUN_VERSION=0.37.2
ARG RERUN_SDK_SHA256=65a84ffa5edda71969432c19b948893502ec3db446f235ba06cf30a46f241a37
RUN curl -sSL -o /tmp/rerun_cpp_sdk.zip "https://github.com/rerun-io/rerun/releases/download/${RERUN_VERSION}/rerun_cpp_sdk-${RERUN_VERSION}-multiplatform.zip" \
    && echo "${RERUN_SDK_SHA256}  /tmp/rerun_cpp_sdk.zip" | sha256sum -c - \
    && unzip -q /tmp/rerun_cpp_sdk.zip -d /opt/src && rm /tmp/rerun_cpp_sdk.zip \
    && cmake -S /opt/src/rerun_cpp_sdk -B /opt/src/rerun_build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/rerun \
    && cmake --build /opt/src/rerun_build --parallel "$(nproc)" && cmake --install /opt/src/rerun_build \
    && rm -rf /opt/src
# Rerun CLI (viewer + gRPC server + web viewer) inside the image: `docker/scripts/run_live.sh` serves the web
# viewer on :9090 so a browser on the host follows a run live without installing anything on the host.
ARG RERUN_CLI_SHA256=49916e528d7e6a5c00794dadcec260dd32b27fd583784d6fc49934d9074b7260
RUN curl -sSL -o /usr/local/bin/rerun "https://github.com/rerun-io/rerun/releases/download/${RERUN_VERSION}/rerun-cli-${RERUN_VERSION}-aarch64-unknown-linux-gnu" \
    && echo "${RERUN_CLI_SHA256}  /usr/local/bin/rerun" | sha256sum -c - && chmod +x /usr/local/bin/rerun
# The repo is bind-mounted from a different owner; git must not refuse it.
RUN git config --global --add safe.directory '*'
ENV CCACHE_DIR=/ccache \
    CMAKE_GENERATOR=Ninja
WORKDIR /workspace
CMD ["sleep", "infinity"]
