#!/usr/bin/env bash
# Inside the container: ATE of a trajectory against TUM RGB-D ground truth.
# Usage: evaluate_tum.sh <TUM_RGBD/rgbd_dataset_freiburg1_desk> <traj.txt> [--sim3]
set -euo pipefail
seq=${1:?dataset subdir}; traj=${2:?trajectory}; shift 2
exec python3 /workspace/scripts/evaluate_ate.py --gt "/datasets/$seq/groundtruth.txt" --gt-format tum --traj "$traj" "$@"
