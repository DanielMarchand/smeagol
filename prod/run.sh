#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
exec ./build/evolve prod/config.yaml "$@"
