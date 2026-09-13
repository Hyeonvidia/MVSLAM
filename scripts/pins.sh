#!/usr/bin/env bash
# Print the effective submodule pin table (tag + commit) — the source of truth for docs/design/07.
cd "$(dirname "$0")/.."
git submodule foreach --quiet 'printf "%-16s %-18s %s\n" "$(basename $sm_path)" "$(git describe --tags --exact-match 2>/dev/null || echo untagged)" "$(git rev-parse --short HEAD)"'
