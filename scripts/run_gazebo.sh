#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 "$ROOT/scripts/build_fleet_world.py"
exec gz sim -r "$ROOT/worlds/fleet_kinematic.sdf" "$@"
