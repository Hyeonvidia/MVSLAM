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
      libopencv-dev python3 python3-numpy time \
    && rm -rf /var/lib/apt/lists/*
# The repo is bind-mounted from a different owner; git must not refuse it.
RUN git config --global --add safe.directory '*'
ENV CCACHE_DIR=/ccache \
    CMAKE_GENERATOR=Ninja
WORKDIR /workspace
CMD ["sleep", "infinity"]
