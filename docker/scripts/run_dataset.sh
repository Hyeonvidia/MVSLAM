#!/usr/bin/env bash
# Inside the container: run the dataset runner on a EuRoC sequence mounted under /datasets.
# Usage: run_dataset.sh <config.yaml> <EuRoC/MH01> [preset] [out.txt] [extra runner args, e.g. --execution offline]
set -euo pipefail
cfg=${1:?config}; seq=${2:?dataset subdir, e.g. EuRoC/MH01}
preset=${3:-docker-release}; out=${4:-/results/$(basename "$seq")_traj.txt}
shift "$(( $# < 4 ? $# : 4 ))"
cd /workspace
exec /usr/bin/time -p "build/$preset/bin/mvslam_dataset_runner" --config "$cfg" --euroc "/datasets/$seq/mav0" --out "$out" "$@"
