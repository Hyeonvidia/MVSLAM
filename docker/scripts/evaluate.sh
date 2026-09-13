#!/usr/bin/env bash
# Inside the container: ATE of a trajectory against EuRoC ground truth.
# Usage: evaluate.sh <EuRoC/MH01> <traj.txt> [--sim3]
set -euo pipefail
seq=${1:?dataset subdir}; traj=${2:?trajectory}; shift 2
exec python3 /workspace/scripts/evaluate_ate.py --gt "/datasets/$seq/mav0/state_groundtruth_estimate0/data.csv" --traj "$traj" "$@"
