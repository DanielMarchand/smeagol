#!/usr/bin/env bash
# run.sh – validate the example robot YAML files
# Run from any directory; the script resolves paths relative to itself.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BINARY="$REPO_ROOT/build/validate_robot"

# ── sanity check ──────────────────────────────────────────────────────────────
if [[ ! -f "$BINARY" ]]; then
    echo "Binary not found: $BINARY"
    echo "Build the project first:  cmake -B build && cmake --build build"
    exit 1
fi

echo "Running validator on example files..."
echo ""

"$BINARY" \
    "$SCRIPT_DIR/good_robot.yaml" \
    "$SCRIPT_DIR/bad_robot.yaml"
