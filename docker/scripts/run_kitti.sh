#!/usr/bin/env bash
# Inside the container: run the dataset runner on a KITTI odometry sequence mounted under /datasets.
# Usage: run_kitti.sh <config.yaml> <sequence NN> [preset] [out.txt] [extra runner args]
# Expects /datasets/kitti_dataset/data_odometry_{gray,calib}/dataset/sequences/NN.
set -euo pipefail
cfg=${1:?config}; seq=${2:?sequence, e.g. 00}
preset=${3:-docker-release}; out=${4:-/results/kitti_${seq}_traj.txt}
shift "$(( $# < 4 ? $# : 4 ))"
root=/datasets/kitti_dataset
cd /workspace
exec /usr/bin/time -p "build/$preset/bin/mvslam_dataset_runner" --config "$cfg" --kitti "$root/data_odometry_gray/dataset/sequences/$seq" \
  --kitti-calib "$root/data_odometry_calib/dataset/sequences/$seq/calib.txt" --out "$out" "$@"
