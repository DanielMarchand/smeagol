#!/usr/bin/env bash
# mutator_demo/run.sh
#
# Systematically applies each of the four mutation operators to seed_robot.yaml,
# producing 5 offspring per operator with deterministic seeds.
# Results are written to robots/ (not committed to git).
#
# Requires: demo_mutator built in ../../build/
#   cd ../../build && make demo_mutator

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/../../build"
TOOL="$BUILD_DIR/demo_mutator"
SNAPSHOT="$BUILD_DIR/snapshot_robot"
INPUT="$SCRIPT_DIR/seed_robot.yaml"
ROBOTS_DIR="$SCRIPT_DIR/robots"

if [[ ! -x "$TOOL" ]]; then
    echo "ERROR: demo_mutator not found at $TOOL"
    echo "       Build it first:  cd ../../build && make demo_mutator"
    exit 1
fi

if [[ ! -x "$SNAPSHOT" ]]; then
    echo "ERROR: snapshot_robot not found at $SNAPSHOT"
    echo "       Build it first:  cd ../../build && make snapshot_robot"
    exit 1
fi

# Helper: render a PNG for every YAML matching a glob pattern
snapshot_batch() {
    local pattern="$1"
    for yaml in $pattern; do
        local png="${yaml%.yaml}.png"
        "$SNAPSHOT" "$yaml" "$png"
        echo "    snapshot → $(basename "$png")"
    done
}

mkdir -p "$ROBOTS_DIR"

echo "======================================================"
echo " Mutator Demo"
echo " Seed: $INPUT"
echo " Output: $ROBOTS_DIR"
echo "======================================================"
echo

# ── 1. perturb (p=0.10) ───────────────────────────────────────────────────────
# Nudges a bar's rest-length by ±10% and/or a neuron's threshold/weight by ±0.5.
echo "--- [1/4] perturb  (bar rest-length / neuron threshold / synapse weight) ---"
"$TOOL" "$INPUT" perturb 5 42 "$ROBOTS_DIR"
snapshot_batch "$ROBOTS_DIR/perturb_*.yaml"
echo

# ── 2. add_remove (p=0.01) ────────────────────────────────────────────────────
# Adds or removes a dangling bar or an unconnected neuron.
echo "--- [2/4] add_remove  (add/remove bar or neuron) ---"
"$TOOL" "$INPUT" add_remove 5 42 "$ROBOTS_DIR"
snapshot_batch "$ROBOTS_DIR/add_remove_*.yaml"
echo

# ── 3. split (p=0.03) ─────────────────────────────────────────────────────────
# Splits a vertex into two (+ tiny bar) or splits a bar at its midpoint.
# Actuators on a split bar are re-routed to the first half.
echo "--- [3/4] split  (vertex→two + bar   |   bar→midpoint vertex + two halves) ---"
"$TOOL" "$INPUT" split 5 42 "$ROBOTS_DIR"
snapshot_batch "$ROBOTS_DIR/split_*.yaml"
echo

# ── 4. attach/detach (p=0.03) ─────────────────────────────────────────────────
# Flips a bar between structural and actuated (toggle, not separate add/remove).
echo "--- [4/5] attach  (flip bar between structural ↔ actuated) ---"
"$TOOL" "$INPUT" attach 5 42 "$ROBOTS_DIR"
snapshot_batch "$ROBOTS_DIR/attach_*.yaml"
echo

# ── 5. rewire (p=0.03) ───────────────────────────────────────────────────────
# Reassigns an actuator's bar target or neural source to a different index.
echo "--- [5/5] rewire  (reassign actuator bar_idx or neuron_idx) ---"
"$TOOL" "$INPUT" rewire 5 42 "$ROBOTS_DIR"
snapshot_batch "$ROBOTS_DIR/rewire_*.yaml"
echo

echo "======================================================"
echo " Done.  $(ls "$ROBOTS_DIR"/*.yaml 2>/dev/null | wc -l) robots + $(ls "$ROBOTS_DIR"/*.png 2>/dev/null | wc -l) snapshots written to:"
echo " $ROBOTS_DIR"
echo "======================================================"
