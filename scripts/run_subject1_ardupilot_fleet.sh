#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AP_ROOT="${AP_ROOT:-$HOME/ardupilot}"
AP_GZ_ROOT="${AP_GZ_ROOT:-$HOME/ardupilot_gazebo}"
BASE_PORT="${BASE_PORT:-9002}"
INSTANCE_COUNT="${INSTANCE_COUNT:-6}"
ALLOW_AP_VERSION_MISMATCH="${ALLOW_AP_VERSION_MISMATCH:-0}"
LOG_DIR="${ROOT}/runtime/ardupilot_fleet"
WORLD_DIR="${LOG_DIR}"
WORLD="${WORLD_DIR}/zhixin_subject1_ardupilot_fleet.sdf"

die() { echo "[subject1-fleet-sitl] ERROR: $*" >&2; exit 1; }
command -v gz >/dev/null || die "missing gz"
[[ -x "${AP_ROOT}/Tools/autotest/sim_vehicle.py" ]] || die "missing sim_vehicle.py: ${AP_ROOT}"
[[ -x "${AP_ROOT}/build/sitl/bin/arducopter" ]] || die "missing ArduCopter SITL: ${AP_ROOT}"
[[ -f "${AP_GZ_ROOT}/build/libArduPilotPlugin.so" ]] || die "missing ArduPilot Gazebo plugin"
[[ "${INSTANCE_COUNT}" == "6" ]] || die "this competition profile requires six instances"

ACTUAL_VERSION="$(cd "${AP_ROOT}" && git describe --tags --always --dirty 2>/dev/null || echo unknown)"
if [[ "${ALLOW_AP_VERSION_MISMATCH}" != "1" && "${ACTUAL_VERSION}" != *"4.7.1"* ]]; then
  die "AP source is ${ACTUAL_VERSION}, expected 4.7.1; use ALLOW_AP_VERSION_MISMATCH=1 only for provisional SITL"
fi

mkdir -p "${LOG_DIR}"
export GZ_VERSION="${GZ_VERSION:-harmonic}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${AP_GZ_ROOT}/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="${WORLD_DIR}/models:${AP_GZ_ROOT}/models:${AP_GZ_ROOT}/worlds:${GZ_SIM_RESOURCE_PATH:-}"
PYTHONPATH="${ROOT}/scripts" python3 "${ROOT}/scripts/build_ardupilot_fleet_world.py" --base-port "${BASE_PORT}"

PIDS=()
cleanup() {
  set +e
  for pid in "${PIDS[@]:-}"; do kill "${pid}" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

gz sim -r "${WORLD}" >"${LOG_DIR}/gazebo.log" 2>&1 &
PIDS+=("$!")
sleep "${GZ_STARTUP_WAIT_S:-5}"

cd "${AP_ROOT}"
for ((instance=0; instance<INSTANCE_COUNT; instance++)); do
  python3 Tools/autotest/sim_vehicle.py \
    -D -N -v ArduCopter -f gazebo-iris \
    -I "${instance}" --no-rebuild --model JSON --no-mavproxy \
    --out "127.0.0.1:$((5760 + instance))" \
    >"${LOG_DIR}/sitl-${instance}.log" 2>&1 &
  PIDS+=("$!")
done

echo "Gazebo world: ${WORLD}"
echo "SITL instances: ${INSTANCE_COUNT}; FDM ports: ${BASE_PORT}..$((BASE_PORT + INSTANCE_COUNT - 1))"
echo "This launcher proves process/topic wiring only until each MAVLink instance reports telemetry and controlled pose change."
wait
