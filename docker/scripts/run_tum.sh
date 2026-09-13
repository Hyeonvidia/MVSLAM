#!/usr/bin/env bash
# Inside the container: run the dataset runner on a TUM RGB-D sequence mounted under /datasets.
# Usage: run_tum.sh <config.yaml> <TUM_RGBD/rgbd_dataset_freiburg1_desk> [preset] [out.txt] [extra runner args]
set -euo pipefail
cfg=${1:?config}; seq=${2:?dataset subdir}
preset=${3:-docker-release}; out=${4:-/results/$(basename "$seq")_traj.txt}
shift "$(( $# < 4 ? $# : 4 ))"
cd /workspace
exec /usr/bin/time -p "build/$preset/bin/mvslam_dataset_runner" --config "$cfg" --tum "/datasets/$seq" --out "$out" "$@"
