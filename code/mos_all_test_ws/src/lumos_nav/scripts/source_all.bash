#!/bin/bash
# Source all environments for lumos_nav system
# Usage: source source_all.bash  (must be sourced, not executed)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"

echo "[lumos_nav] Sourcing /opt/slam environment..."
if [ -f /opt/slam/apps/install/setup.bash ]; then
    source /opt/slam/apps/install/setup.bash
else
    echo "[WARN] /opt/slam/apps/install/setup.bash not found"
fi

echo "[lumos_nav] Sourcing 3dvision1 workspace environment..."
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
else
    echo "[WARN] $PROJECT_ROOT/install/setup.bash not found"
fi

# Set radar type (change if needed: MID360, MID360s, avia, etc.)
export SLAM_LIDAR1_NAME=MID360
echo "[lumos_nav] SLAM_LIDAR1_NAME=$SLAM_LIDAR1_NAME"

# Verify correct package resolution from this workspace.
echo "[lumos_nav] Verifying lumos_nav package location..."
python3 - "$PROJECT_ROOT" <<'PY'
import sys
from ament_index_python.packages import get_package_share_directory

pkg = get_package_share_directory('lumos_nav')
project_root = sys.argv[1]
if pkg.startswith(f'{project_root}/install/'):
    print(f'  OK: {pkg}')
else:
    print(f'  WARN: found at {pkg} (expected {project_root}/install/...)')
PY

echo "[lumos_nav] Environment ready."
