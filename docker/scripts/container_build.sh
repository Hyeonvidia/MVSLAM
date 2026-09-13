#!/usr/bin/env bash
# Inside the container: configure + build + test one preset (default docker-release).
set -euo pipefail
preset=${1:-docker-release}
cd /workspace
echo "== configure ($preset)"; cmake --preset "$preset"
echo "== build";              cmake --build --preset "$preset" --parallel "$(nproc)"
echo "== test";               ctest --preset "$preset"
ccache -s 2>/dev/null | head -5 || true
