#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AP_ROOT="${AP_ROOT:-$HOME/ardupilot}"
AP_GZ_ROOT="${AP_GZ_ROOT:-$HOME/ardupilot_gazebo}"
AP_VERSION_EXPECTED="${AP_VERSION_EXPECTED:-4.7.1}"
PORT="${PORT:-8766}"
INSTANCE="${INSTANCE:-0}"
LOG_DIR="${ROOT}/runtime/ardupilot"
mkdir -p "${LOG_DIR}"

die() { echo "[subject1-sitl] ERROR: $*" >&2; exit 1; }

command -v gz >/dev/null || die "未找到 gz；请先安装 Gazebo Harmonic。"
[[ -x "${AP_ROOT}/Tools/autotest/sim_vehicle.py" ]] || die "未找到 sim_vehicle.py: ${AP_ROOT}"
[[ -x "${AP_ROOT}/build/sitl/bin/arducopter" ]] || die "未找到已编译的 ArduCopter SITL: ${AP_ROOT}/build/sitl/bin/arducopter"
[[ -f "${AP_GZ_ROOT}/build/libArduPilotPlugin.so" ]] || die "未找到 ArduPilot Gazebo 插件: ${AP_GZ_ROOT}/build/libArduPilotPlugin.so"

ACTUAL_VERSION="$(cd "${AP_ROOT}" && git describe --tags --always --dirty 2>/dev/null || echo unknown)"
if [[ "${ALLOW_AP_VERSION_MISMATCH:-0}" != "1" && "${ACTUAL_VERSION}" != *"${AP_VERSION_EXPECTED}"* ]]; then
  die "当前AP源码为 ${ACTUAL_VERSION}，不是期望的 ${AP_VERSION_EXPECTED}；如仅做临时SITL验证，请设置 ALLOW_AP_VERSION_MISMATCH=1。"
fi

export GZ_VERSION="${GZ_VERSION:-harmonic}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${AP_GZ_ROOT}/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="${AP_GZ_ROOT}/models:${AP_GZ_ROOT}/worlds:${GZ_SIM_RESOURCE_PATH:-}"

WORLD="${AP_GZ_ROOT}/worlds/iris_runway.sdf"
[[ -f "${WORLD}" ]] || die "缺少示例世界：${WORLD}"

cleanup() {
  set +e
  [[ -n "${SITL_PID:-}" ]] && kill "${SITL_PID}" 2>/dev/null
  [[ -n "${GZ_PID:-}" ]] && kill "${GZ_PID}" 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "[subject1-sitl] AP=${ACTUAL_VERSION} GZ=${GZ_VERSION} instance=${INSTANCE}"
echo "[subject1-sitl] starting Gazebo: ${WORLD}"
gz sim -r "${WORLD}" >"${LOG_DIR}/gazebo.log" 2>&1 &
GZ_PID=$!
sleep "${GZ_STARTUP_WAIT_S:-3}"
kill -0 "${GZ_PID}" 2>/dev/null || die "Gazebo启动失败，查看 ${LOG_DIR}/gazebo.log"

echo "[subject1-sitl] starting one ArduCopter JSON SITL"
cd "${AP_ROOT}"
python3 Tools/autotest/sim_vehicle.py \
  -D -N -v ArduCopter -f gazebo-iris \
  -I "${INSTANCE}" --no-rebuild \
  --model JSON --no-mavproxy \
  --out "127.0.0.1:$((5760 + INSTANCE))" \
  >"${LOG_DIR}/sitl-${INSTANCE}.log" 2>&1 &
SITL_PID=$!

echo "[subject1-sitl] SITL pid=${SITL_PID}; logs: ${LOG_DIR}"
echo "[subject1-sitl] 注意：这是单机插件冒烟测试，不是六机比赛闭环。"
wait "${SITL_PID}"
