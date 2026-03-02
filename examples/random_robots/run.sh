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

PIDS=()
TMPS=()
NAMES=()
OUTS=()

for ROBOT in "${ROBOTS[@]}"; do
    NAME="$(basename "$ROBOT" .yaml)"
    OUT="${MP4_BASE}_${NAME}.mp4"

    TMPPARAMS="$(mktemp /tmp/simulate_XXXXXX.yaml)"
    sed "s|^output:.*|output: $OUT|" "$SIMULATE_CFG" > "$TMPPARAMS"

    echo "  Launching $NAME → $OUT"
    "$EVAL_BIN" "$ROBOT" "$TMPPARAMS" < /dev/null \
        > "/tmp/smeagol_log_${NAME}.txt" 2>&1 &

    PIDS+=($!)
    TMPS+=("$TMPPARAMS")
    NAMES+=("$NAME")
    OUTS+=("$OUT")
done

echo ""
echo "All ${#PIDS[@]} robots running in parallel (PIDs: ${PIDS[*]})"
echo ""

# Wait for each job and report pass/fail as it finishes.
ALL_OK=1
for i in "${!PIDS[@]}"; do
    if wait "${PIDS[$i]}"; then
        echo "  ✓  ${NAMES[$i]} done  →  ${OUTS[$i]}"
    else
        echo "  ✗  ${NAMES[$i]} FAILED  (log: /tmp/smeagol_log_${NAMES[$i]}.txt)"
        ALL_OK=0
    fi
    rm -f "${TMPS[$i]}"
done

echo ""
if [[ $ALL_OK -eq 1 ]]; then
    echo "All robots completed.  Videos:"
    for OUT in "${OUTS[@]}"; do echo "  ffplay $OUT"; done
else
    echo "Some robots failed — check /tmp/smeagol_log_*.txt for details."
fi
