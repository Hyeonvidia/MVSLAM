#!/usr/bin/env bash
# Host-side wrapper — the only way this project is built and tested (no host builds).
#   scripts/docker.sh image                  build the dev image
#   scripts/docker.sh build [preset]         configure + build + ctest inside the container (default docker-release)
#   scripts/docker.sh test  [preset]         ctest only
#   scripts/docker.sh run <config> <EuRoC/MH01> [preset] [out] [runner args]   EuRoC runner
#   scripts/docker.sh kitti <config> <NN> [preset] [out] [runner args]         KITTI odometry runner
#   scripts/docker.sh tum <config> <TUM_RGBD/seq> [preset] [out] [runner args]    TUM RGB-D runner
#   scripts/docker.sh eval <EuRoC/MH01> <traj> [--sim3] | eval-kitti <NN> <traj> | eval-tum <TUM_RGBD/seq> <traj>
#   scripts/docker.sh eval-loc <ref_traj> <loc.txt> [EuRoC/V101]                   relocalization success rate
#   scripts/docker.sh shell                  interactive shell
#   scripts/docker.sh clean                  remove build/ccache volumes
set -euo pipefail
cd "$(dirname "$0")/.."
cmd=${1:-help}; shift || true
case "$cmd" in
  image) docker compose build ;;
  build) docker compose run --rm dev docker/scripts/container_build.sh "${1:-docker-release}" ;;
  test)  docker compose run --rm dev ctest --preset "${1:-docker-release}" ;;
  tsan)  docker compose run --rm dev-tsan docker/scripts/container_build.sh docker-tsan ;;
  run)   docker compose run --rm dev docker/scripts/run_dataset.sh "$@" ;;
  kitti) docker compose run --rm dev docker/scripts/run_kitti.sh "$@" ;;
  eval)  docker compose run --rm dev docker/scripts/evaluate.sh "$@" ;;
  eval-kitti) docker compose run --rm dev docker/scripts/evaluate_kitti.sh "$@" ;;
  tum)   docker compose run --rm dev docker/scripts/run_tum.sh "$@" ;;
  eval-tum) docker compose run --rm dev docker/scripts/evaluate_tum.sh "$@" ;;
  eval-loc) docker compose run --rm dev docker/scripts/evaluate_localization.sh "$@" ;;
  shell) docker compose run --rm dev bash "$@" ;;
  clean) docker compose down -v ;;
  *) sed -n '2,14p' "$0" ;;
esac
