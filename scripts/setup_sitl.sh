#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
target="$PWD/vendor/ardupilot-4.7.1"
if [[ ! -d "$target/.git" ]]; then
    git clone --branch Copter-4.7.1 --recurse-submodules https://github.com/ArduPilot/ardupilot.git "$target"
fi
if [[ "$(git -C "$target" rev-parse HEAD)" != "$(git -C "$target" rev-parse 'Copter-4.7.1^{commit}')" ]]; then
    printf '%s\n' '目录不是Copter-4.7.1；不自动重置用户源码' >&2
    exit 1
fi
cd "$target"
git submodule update --init --recursive
./waf configure --board sitl
./waf copter -j "${JOBS:-4}"
