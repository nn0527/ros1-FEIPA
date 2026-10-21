#!/bin/sh
# Local offline preview; never connects to a flight controller or Raspberry Pi.
set -eu
cd "$(dirname "$0")"
exec python3 -B src/px4_wall_ceiling_control/scripts/operator/ceiling_wall_console.py --bridge mock --port 8791 --open-browser "$@"
