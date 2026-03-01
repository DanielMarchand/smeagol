#!/usr/bin/env bash
# run.sh – deformation series sanity-check for §3.1 (energy function)
#
# Pushes the apex vertex (vertex 3) of the tetrahedron upward in Z by 0.02 m
# per step over 10 steps (0 → 0.20 m above rest position).
#
# Expected output:
#   step 0  : H_elastic ≈ 0  (robot exactly at rest geometry)
#   step 1+ : H_elastic rises smoothly as all three apex struts stretch
#   H_gravity rises linearly (apex gains height each step)
#
# Snapshots saved to: /tmp/golem_deform/
# Requires a display server (X11).  On headless: DISPLAY=:99 ./run.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BINARY="$REPO_ROOT/build/deformation_series"
YAML="$SCRIPT_DIR/deform_robot.yaml"
OUTDIR="/tmp/golem_deform"

if [[ ! -f "$BINARY" ]]; then
    echo "Binary not found: $BINARY"
    echo "Build first:  cmake -B build && cmake --build build"
    exit 1
fi

echo "Running deformation series..."
echo "  robot  : $YAML"
echo "  output : $OUTDIR"
echo ""

#                 yaml    outdir    v  dx dy dz  step   steps
"$BINARY"  "$YAML"  "$OUTDIR"  3  0  0  1  0.02   10

echo ""
echo "View snapshots:"
echo "  eog $OUTDIR/          # Eye of GNOME (gallery view)"
echo "  xdg-open $OUTDIR/step_00.png  # step 0 (at rest)"
echo "  xdg-open $OUTDIR/step_10.png  # step 10 (most deformed)"
