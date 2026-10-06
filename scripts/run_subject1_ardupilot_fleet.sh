#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AP_ROOT="${AP_ROOT:-$HOME/ardupilot}"
AP_GZ_ROOT="${AP_GZ_ROOT:-$HOME/ardupilot_gazebo}"
BASE_PORT="${BASE_PORT:-9002}"
PORT_STEP="${PORT_STEP:-10}"
INSTANCE_COUNT="${INSTANCE_COUNT:-6}"
ALLOW_AP_VERSION_MISMATCH="${ALLOW_AP_VERSION_MISMATCH:-0}"
LOG_DIR="${ROOT}/runtime/ardupilot_fleet"
WORLD_DIR="${LOG_DIR}"
WORLD="${WORLD_DIR}/zhixin_subject1_ardupilot_fleet.sdf"

die() { echo "[subject1-fleet-sitl] ERROR: $*" >&2; exit 1; }
command -v gz >/dev/null || die "missing gz"
[[ -x "${AP_ROOT}/build/sitl/bin/arducopter" ]] || die "missing ArduCopter SITL: ${AP_ROOT}"
[[ -f "${AP_GZ_ROOT}/build/libArduPilotPlugin.so" ]] || die "missing ArduPilot Gazebo plugin"
MAVPROXY_BIN="${MAVPROXY_BIN:-${HOME}/.local/bin/mavproxy.py}"
[[ -x "${MAVPROXY_BIN}" ]] || die "missing MAVProxy: ${MAVPROXY_BIN}"
[[ "${INSTANCE_COUNT}" == "6" ]] || die "this competition profile requires six instances"

ACTUAL_VERSION="$(cd "${AP_ROOT}" && git describe --tags --always --dirty 2>/dev/null || echo unknown)"
if [[ "${ALLOW_AP_VERSION_MISMATCH}" != "1" && "${ACTUAL_VERSION}" != *"4.7.1"* ]]; then
  die "AP source is ${ACTUAL_VERSION}, expected 4.7.1; use ALLOW_AP_VERSION_MISMATCH=1 only for provisional SITL"
fi

mkdir -p "${LOG_DIR}"
export GZ_VERSION="${GZ_VERSION:-harmonic}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${AP_GZ_ROOT}/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="${WORLD_DIR}/models:${AP_GZ_ROOT}/models:${AP_GZ_ROOT}/worlds:${GZ_SIM_RESOURCE_PATH:-}"
PYTHONPATH="${ROOT}/scripts" python3 "${ROOT}/scripts/build_ardupilot_fleet_world.py" --base-port "${BASE_PORT}" --port-step "${PORT_STEP}"

PIDS=()
cleanup() {
  set +e
  for pid in "${PIDS[@]:-}"; do kill "${pid}" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

gz sim -r "${WORLD}" >"${LOG_DIR}/gazebo.log" 2>&1 &
PIDS+=("$!")
sleep "${GZ_STARTUP_WAIT_S:-5}"

for ((instance=0; instance<INSTANCE_COUNT; instance++)); do
  MAVLINK_PORT=$((5760 + instance * 10))
  TELEMETRY_PORT=$((14550 + instance * 10))
  nohup "${AP_ROOT}/build/sitl/bin/arducopter" \
    --model JSON --speedup "${SITL_SPEEDUP:-1}" --slave 0 \
    --sim-address=127.0.0.1 -I "${instance}" --sysid "$((instance + 1))" \
    --defaults "${AP_ROOT}/Tools/autotest/default_params/gazebo-iris.parm" \
    >"${LOG_DIR}/sitl-${instance}.log" 2>&1 < /dev/null &
  PIDS+=("$!")
  sleep 0.5
  nohup "${MAVPROXY_BIN}" \
    --master="tcp:127.0.0.1:${MAVLINK_PORT}" \
    --out="udp:127.0.0.1:${TELEMETRY_PORT}" \
    --daemon --non-interactive \
    >"${LOG_DIR}/mavproxy-${instance}.log" 2>&1 < /dev/null &
  PIDS+=("$!")
done

echo "Gazebo world: ${WORLD}"
echo "SITL instances: ${INSTANCE_COUNT}; FDM ports: ${BASE_PORT}, $((BASE_PORT + PORT_STEP)), ... $((BASE_PORT + (INSTANCE_COUNT - 1) * PORT_STEP))"
echo "MAVLink masters: tcp://127.0.0.1:5760, tcp://127.0.0.1:5770, ... tcp://127.0.0.1:5810"
echo "MAVProxy telemetry outputs: UDP 127.0.0.1:14550, 14560, ... 14600"
echo "This launcher proves process/topic wiring only until each MAVLink instance reports telemetry and controlled pose change."
wait
