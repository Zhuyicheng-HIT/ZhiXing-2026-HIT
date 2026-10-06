#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${PORT:-8765}"
mkdir -p "$ROOT/runtime"

command -v gz >/dev/null || { echo 'missing Gazebo gz'; exit 1; }
python3 "$ROOT/scripts/build_fleet_world.py"

gz sim -r "$ROOT/worlds/fleet_kinematic.sdf" >"$ROOT/runtime/gazebo.log" 2>&1 &
GAZEBO_PID=$!
(cd "$ROOT" && python3 scripts/gazebo_image_bridge.py --period-s 0.25 >runtime/gazebo_image_bridge.log 2>&1) &
BRIDGE_PID=$!

cleanup() {
  kill "$BRIDGE_PID" "$GAZEBO_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Gazebo PID: $GAZEBO_PID"
echo "Camera bridge PID: $BRIDGE_PID"
echo "Open from Windows: http://localhost:${PORT}"
cd "$ROOT"
exec python3 scripts/sim_server.py --host 0.0.0.0 --port "$PORT"
