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

# good_robot.yaml must pass — non-zero exit is a real failure
echo "==> good_robot.yaml  (expect: VALID)"
"$BINARY" "$SCRIPT_DIR/good_robot.yaml"

echo ""

# bad_robot.yaml must fail — a zero exit would mean validation is broken
echo "==> bad_robot.yaml  (expect: INVALID)"
if "$BINARY" "$SCRIPT_DIR/bad_robot.yaml"; then
    echo "ERROR: bad_robot.yaml was accepted as valid — validation logic is broken"
    exit 1
fi

echo ""
echo "All validation checks passed."
