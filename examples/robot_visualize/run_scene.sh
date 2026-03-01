#!/usr/bin/env bash
# run_scene.sh – open an interactive 3D window for good_robot.yaml
# Run from any directory; paths are resolved relative to this script.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BINARY="$REPO_ROOT/build/visualize_robot"
YAML="$SCRIPT_DIR/good_robot.yaml"

if [[ ! -f "$BINARY" ]]; then
    echo "Binary not found: $BINARY"
    echo "Build first:  cmake -B build && cmake --build build"
    exit 1
fi

if [[ ! -f "$YAML" ]]; then
    echo "YAML file not found: $YAML"
    exit 1
fi

echo "Launching interactive visualizer..."
echo "  robot : $YAML"
echo "  binary: $BINARY"
echo ""
echo "Controls: orbital camera auto-spins | scroll to zoom | ESC to quit"
echo ""

"$BINARY" "$YAML"
