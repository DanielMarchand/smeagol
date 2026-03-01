#!/usr/bin/env bash
# run.sh – 2-neuron anti-phase oscillator driving bar 3 (left strut).
#
# Expected stdout (every 10 neural cycles):
#   neurons=[0,1]  → bar 3 shortened by 1 cm  (neuron 1 just fired)
#   neurons=[1,0]  → bar 3 lengthened by 1 cm  (neuron 0 just fired)
#   alternates perfectly with period 2
#
# Requires a display server.  On headless:
#   Xvfb :99 -screen 0 640x480x24 &
#   DISPLAY=:99 ./run.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BINARY="$REPO_ROOT/build/fall_animation"

if [[ ! -x "$BINARY" ]]; then
    echo "Binary not found: $BINARY"
    echo "Run:  cmake --build $REPO_ROOT/build --target fall_animation"
    exit 1
fi

"$BINARY" "$SCRIPT_DIR/robot.yaml" "$SCRIPT_DIR/simulation.yaml"

OUTPUT=$(grep '^output:' "$SCRIPT_DIR/simulation.yaml" | awk '{print $2}')
echo "Play: ffplay $OUTPUT"
