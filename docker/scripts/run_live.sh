#!/usr/bin/env bash
# Inside the container: serve the Rerun web viewer (http://localhost:9090 on the host, needs
# `docker compose run --service-ports`) and stream a dataset run into it at the dataset's real rate.
# Usage: run_live.sh (euroc <EuRoC/MH01> | kitti <NN> | tum <TUM_RGBD/seq>) <config.yaml> [preset] [pace] [extra runner args]
# The viewer keeps serving after the run until Ctrl-C so the recording can still be inspected.
set -euo pipefail
kind=${1:?euroc|kitti|tum}; seq=${2:?dataset}; cfg=${3:?config}
preset=${4:-docker-release}; pace=${5:-1.0}
shift "$(( $# < 5 ? $# : 5 ))"
cd /workspace
rerun --serve-web --port 9876 --web-viewer-port 9090 --bind 0.0.0.0 > /results/rerun_server.log 2>&1 &
viewer=$!
trap 'kill $viewer 2>/dev/null || true' EXIT
sleep 2
echo "Rerun web viewer: open http://localhost:9090 in a browser on the host"
case "$kind" in
  euroc) src=(--euroc "/datasets/$seq/mav0") ;;
  kitti) root=/datasets/kitti_dataset; src=(--kitti "$root/data_odometry_gray/dataset/sequences/$seq" --kitti-calib "$root/data_odometry_calib/dataset/sequences/$seq/calib.txt") ;;
  tum)   src=(--tum "/datasets/$seq") ;;
  *) echo "unknown dataset kind $kind" >&2; exit 2 ;;
esac
"build/$preset/bin/mvslam_dataset_runner" --config "$cfg" "${src[@]}" --out "/results/live_traj.txt" --execution realtime --pace "$pace" \
  --rerun rerun+http://127.0.0.1:9876/proxy --rerun-image-every 4 "$@"
echo "run finished; the web viewer stays up (Ctrl-C to stop)"
wait $viewer
