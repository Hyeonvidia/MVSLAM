#!/usr/bin/env bash
# Inside the container: ATE of a trajectory against KITTI odometry ground truth.
# Usage: evaluate_kitti.sh <sequence NN> <traj.txt> [--sim3]
set -euo pipefail
seq=${1:?sequence}; traj=${2:?trajectory}; shift 2
root=/datasets/kitti_dataset
exec python3 /workspace/scripts/evaluate_ate.py --gt "$root/data_odometry_poses/dataset/poses/$seq.txt" \
  --times "$root/data_odometry_gray/dataset/sequences/$seq/times.txt" --traj "$traj" "$@"
