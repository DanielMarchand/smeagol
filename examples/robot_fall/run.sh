#!/usr/bin/env bash
# run.sh – tetrahedron fall animation
# Edit robot.yaml to change geometry; edit simulation.yaml to change physics params.
#
# Requires a display server.  On headless:
#   Xvfb :99 -screen 0 640x480x24 &
#   DISPLAY=:99 ./run.sh [output.mp4]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BINARY="$REPO_ROOT/build/run_simulation"
if [[ ! -x "$BINARY" ]]; then
    echo "Binary not found: $BINARY"
    echo "Run:  cmake --build $REPO_ROOT/build --target run_simulation"
    exit 1
fi

"$BINARY" "$SCRIPT_DIR/robot_upsidedown.yaml" "$SCRIPT_DIR/simulation.yaml"

echo "Play: ffplay \$(grep '^output:' \"$SCRIPT_DIR/simulation.yaml\" | awk '{print \$2}')"
