#!/usr/bin/env bash

set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export REMOTE_START="${REMOTE_TELEOP_START:-/home/lumos/mos_fleet_monitor/bin/mos-teleop-start}"
exec "${SCRIPT_DIR}/start-robot-monitor.sh" "$@"
