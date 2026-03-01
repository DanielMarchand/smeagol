#!/usr/bin/env bash
# run.sh – sine-actuator body physics test
# Bar 3 (left strut) oscillates at 0.5 Hz, ±4 cm amplitude.
# The apex should rock back and forth for 3 full cycles.
#
# Edit robot.yaml to change actuator params (bar_idx, amplitude, frequency).
# Edit simulation.yaml to change fps, steps_per_frame, step_size, output path.
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
