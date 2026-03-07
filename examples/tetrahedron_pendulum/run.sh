#!/usr/bin/env bash
# run.sh – tetrahedron with pendulum bar physics test
#
# A regular tetrahedron (edge 0.2 m) sits on the floor.  An extra "pendulum"
# bar (bar 6) hangs from the apex (v3) with its free end (v4) inside the body,
# angled slightly toward the left strut (v0–v3).
#
# DebugActuator: bar 3 (left strut, v0→apex) oscillates at 1 Hz, ±4 cm.
# This rocks the body asymmetrically each cycle.  Because the simulator is
# quasi-static (no inertia), the free vertex v4 always hangs straight down
# below the apex — no free-swinging pendulum.  The physics question is
# whether the rocking + floor friction produces net forward translation.
#
# Edit robot.yaml to tune amplitude / frequency / pendulum position.
# Edit simulation.yaml to change fps, step count, output path.
#
# Requires a display server.  On headless:
#   Xvfb :99 -screen 0 640x480x24 &
#   DISPLAY=:99 ./run.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BINARY="$REPO_ROOT/build/run_simulation"

if [[ ! -x "$BINARY" ]]; then
    echo "Binary not found: $BINARY"
    echo "Run:  cmake --build $REPO_ROOT/build --target run_simulation"
    exit 1
fi

"$BINARY" "$SCRIPT_DIR/robot.yaml" "$SCRIPT_DIR/simulation.yaml"

OUTPUT=$(grep '^output:' "$SCRIPT_DIR/simulation.yaml" | awk '{print $2}')
echo "Play: ffplay $OUTPUT"
