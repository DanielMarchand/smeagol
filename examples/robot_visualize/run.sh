#!/usr/bin/env bash
# run.sh – visualize the example good_robot.yaml
# Run from any directory; the script resolves paths relative to itself.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BINARY="$REPO_ROOT/build/visualize_robot"
YAML="$SCRIPT_DIR/good_robot.yaml"

# ── sanity checks ─────────────────────────────────────────────────────────────
if [[ ! -f "$BINARY" ]]; then
    echo "Binary not found: $BINARY"
    echo "Build the project first:  cmake -B build && cmake --build build"
    exit 1
fi

if [[ ! -f "$YAML" ]]; then
    echo "YAML file not found: $YAML"
    exit 1
fi

echo "Launching visualizer..."
echo "  robot : $YAML"
echo "  binary: $BINARY"
echo ""
echo "Controls: left-drag to orbit | scroll to zoom | ESC to quit"
echo ""

"$BINARY" "$YAML"
