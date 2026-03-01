#!/usr/bin/env bash
# run_createimage.sh – render good_robot.yaml to a PNG snapshot and exit
# Run from any directory; paths are resolved relative to this script.
#
# Output: /tmp/golem_snapshot_example.png
# Requires a display server.  On a headless machine:
#   Xvfb :99 -screen 0 1280x720x24 &
#   DISPLAY=:99 ./run_createimage.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BINARY="$REPO_ROOT/build/snapshot_robot"
YAML="$SCRIPT_DIR/good_robot.yaml"
OUTPUT="/tmp/golem_snapshot_example.png"

if [[ ! -f "$BINARY" ]]; then
    echo "Binary not found: $BINARY"
    echo "Build first:  cmake -B build && cmake --build build"
    exit 1
fi

if [[ ! -f "$YAML" ]]; then
    echo "YAML file not found: $YAML"
    exit 1
fi

echo "Rendering snapshot..."
echo "  robot : $YAML"
echo "  output: $OUTPUT"
echo ""

"$BINARY" "$YAML" "$OUTPUT"

echo ""
echo "Done. Open with:  xdg-open $OUTPUT"
