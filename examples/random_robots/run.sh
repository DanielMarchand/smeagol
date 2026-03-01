#!/usr/bin/env bash
# run.sh – generate a batch of random robots and simulate each one.
#
# No arguments — all settings live in generate.yaml and simulate.yaml.
#
# Step 1: generate_robots reads generate.yaml → writes robots/robot_N.yaml
# Step 2: evaluate_fitness simulates every robot and records an MP4

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

GEN_BIN="$REPO_ROOT/build/generate_robots"
EVAL_BIN="$REPO_ROOT/build/evaluate_fitness"

# ── Check binaries ────────────────────────────────────────────────────────────
for BIN in "$GEN_BIN" "$EVAL_BIN"; do
    if [[ ! -f "$BIN" ]]; then
        echo "Binary not found: $BIN"
        echo "Build:  cmake -B $REPO_ROOT/build && cmake --build $REPO_ROOT/build"
        exit 1
    fi
done

GENERATE_CFG="$SCRIPT_DIR/generate.yaml"
SIMULATE_CFG="$SCRIPT_DIR/simulate.yaml"
ROBOTS_DIR="$SCRIPT_DIR/robots"

# ── Step 1: Generate robots ───────────────────────────────────────────────────
echo "============================================================"
echo " Step 1: Generating robots  ($GENERATE_CFG)"
echo "============================================================"
"$GEN_BIN" "$GENERATE_CFG" < /dev/null

# ── Step 2: Simulate every generated robot ───────────────────────────────────
echo ""
ROBOTS=( "$ROBOTS_DIR"/robot_*.yaml )
if [[ ! -f "${ROBOTS[0]}" ]]; then
    echo "No robot YAML files found in $ROBOTS_DIR"
    exit 1
fi

OUTPUT_MP4="$(grep '^output:' "$SIMULATE_CFG" | awk '{print $2}')"
# Strip extension to build per-robot output paths
MP4_BASE="${OUTPUT_MP4%.mp4}"

for ROBOT in "${ROBOTS[@]}"; do
    NAME="$(basename "$ROBOT" .yaml)"
    OUT="${MP4_BASE}_${NAME}.mp4"

    # Temporarily override the output field so each robot gets its own MP4.
    # We write a patched config to a temp file rather than modifying simulate.yaml.
    TMPPARAMS="$(mktemp /tmp/simulate_XXXXXX.yaml)"
    sed "s|^output:.*|output: $OUT|" "$SIMULATE_CFG" > "$TMPPARAMS"

    echo "============================================================"
    echo " Step 2: Simulating $NAME  ($SIMULATE_CFG)"
    echo "============================================================"
    "$EVAL_BIN" "$ROBOT" "$TMPPARAMS" < /dev/null
    rm -f "$TMPPARAMS"

    echo "  Video: ffplay $OUT"
    echo ""
done
