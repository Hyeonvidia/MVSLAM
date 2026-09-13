#!/usr/bin/env bash
# Inside the container: relocalization success rate against the mapping run's trajectory.
# Usage: evaluate_localization.sh <ref_traj.txt> <loc.txt> [EuRoC/V101] [extra args]
set -euo pipefail
ref=${1:?reference trajectory}; loc=${2:?localization file}; seq=${3:-}
shift "$(( $# < 3 ? $# : 3 ))"
gt=()
if [ -n "$seq" ]; then gt=(--gt "/datasets/$seq/mav0/state_groundtruth_estimate0/data.csv"); fi
exec python3 /workspace/scripts/evaluate_localization.py --ref "$ref" --loc "$loc" "${gt[@]}" "$@"
