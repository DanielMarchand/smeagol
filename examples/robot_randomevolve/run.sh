#!/usr/bin/env bash
# run.sh — drift a robot through mutation space without fitness evaluation.
# All settings live in drift.yaml next to this script.
#
# Usage (from anywhere):
#   examples/drift_robot/run.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BIN="$REPO_ROOT/build/drift_robot"

if [[ ! -f "$BIN" ]]; then
    echo "Binary not found: $BIN"
    echo "Build:  cmake -B $REPO_ROOT/build && cmake --build $REPO_ROOT/build"
    exit 1
fi

"$BIN" "$SCRIPT_DIR/drift.yaml"
