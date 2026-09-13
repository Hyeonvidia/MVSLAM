#!/usr/bin/env bash
# Fetch all pinned submodules shallowly (first checkout or after a pin bump).
set -euo pipefail
cd "$(dirname "$0")/.."
git submodule sync --recursive
git submodule update --init --recursive --depth 1 --jobs 4
git submodule status
