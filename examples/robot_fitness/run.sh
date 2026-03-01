#!/usr/bin/env bash
# run.sh – evaluate the tripod walker and record a video.
# Run from any directory; paths are resolved relative to this script.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BINARY="$REPO_ROOT/build/evaluate_fitness"

if [[ ! -f "$BINARY" ]]; then
    echo "Binary not found: $BINARY"
    echo "Build:  cmake -B $REPO_ROOT/build && cmake --build $REPO_ROOT/build --target evaluate_fitness"
    exit 1
fi

PARAMS="$SCRIPT_DIR/params.yaml"
ROBOT="$SCRIPT_DIR/robot.yaml"

echo "============================================================"
echo " Fitness evaluation — tripod walker"
echo "============================================================"
"$BINARY" "$ROBOT" "$PARAMS" < /dev/null

OUTPUT="$(grep '^output:' "$PARAMS" | awk '{print $2}')"
echo ""
echo "Play video:  ffplay $OUTPUT"
