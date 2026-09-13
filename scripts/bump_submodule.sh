#!/usr/bin/env bash
# Usage: scripts/bump_submodule.sh <name> <tag>   e.g. scripts/bump_submodule.sh ceres-solver 2.3.0
# Moves third_party/<name> to <tag> (shallow) and stages the change. Record the reason in docs/adr/.
set -euo pipefail
name=$1; tag=$2; dir="third_party/$name"
cd "$(dirname "$0")/.."
git -C "$dir" fetch --depth 1 origin "refs/tags/${tag}:refs/tags/${tag}"
git -C "$dir" checkout -q "${tag}"
git add "$dir"
echo "$name -> $tag ($(git -C "$dir" rev-parse --short HEAD)); now update the pin table in docs/design/07-build-platforms.md"
