#!/bin/bash
# ============================================================
# 3dvision1 Nav2 + FAST-LIO localization isolated environment
# Usage: source /root/3dvision1/setup_nav2_fastlio.sh
# Exit:  source /root/3dvision1/unsetup_nav2.sh
# ============================================================

if [ -n "${ZSH_VERSION}" ]; then
    _NAV2_SETUP_SUFFIX="zsh"
elif [ -n "${BASH_VERSION}" ]; then
    _NAV2_SETUP_SUFFIX="bash"
else
    _NAV2_SETUP_SUFFIX="sh"
fi

if [ -n "${ZSH_VERSION}" ]; then
    eval '_NAV2_SCRIPT_PATH=${(%):-%N}'
else
    _NAV2_SCRIPT_PATH="${BASH_SOURCE[0]}"
fi
NAV2_PROJECT_ROOT="$(cd "$(dirname "${_NAV2_SCRIPT_PATH}")" && pwd)"

_nav2_source_setup() {
    _nav2_setup_script="$1/setup.${_NAV2_SETUP_SUFFIX}"
    if [ ! -f "${_nav2_setup_script}" ]; then
        echo "[3dvision1] setup script not found: ${_nav2_setup_script}" >&2
        return 1
    fi
    . "${_nav2_setup_script}"
}

_nav2_conflicting_slam_processes() {
    pgrep -af 'slam_mapping\.sh|slam_relocalization\.sh|slam_relocalization_and_navigation\.sh|livox_ros_driver2_node|fastlio_mapping|gicp_localization|transform_fusion\.py|purepursuit_follower' 2>/dev/null
}

if _nav2_conflicting_slam_processes >/tmp/nav2_fastlio_conflicts.$$; then
    if [ -s /tmp/nav2_fastlio_conflicts.$$ ]; then
        echo "[3dvision1] killing leftover nav2/slam processes..." >&2
        cat /tmp/nav2_fastlio_conflicts.$$ >&2
        _nav2_conflicting_slam_processes | awk '{print $1}' | xargs -r kill -9 2>/dev/null
        sleep 1
        echo "[3dvision1] stale processes cleaned, continuing..." >&2
        rm -f /tmp/nav2_fastlio_conflicts.$$
    fi
fi
rm -f /tmp/nav2_fastlio_conflicts.$$

# Save current environment
export _NAV2_SAVED_AMENT_PREFIX="${AMENT_PREFIX_PATH}"
export _NAV2_SAVED_ROS_DOMAIN="${ROS_DOMAIN_ID}"
export _NAV2_SAVED_ROS_LOCALHOST="${ROS_LOCALHOST_ONLY}"
export _NAV2_SAVED_LD_LIBRARY="${LD_LIBRARY_PATH}"
export _NAV2_SAVED_PATH="${PATH}"
export _NAV2_SAVED_PYTHONPATH="${PYTHONPATH}"
export _NAV2_SAVED_CMAKE_PREFIX="${CMAKE_PREFIX_PATH}"
export _NAV2_SAVED_COLCON_PREFIX="${COLCON_PREFIX_PATH}"

# Share the domain with mos_arm_controller / keyboard teleop so /cmd_vel reaches the chassis.
export ROS_DOMAIN_ID=111
export ROS_LOCALHOST_ONLY=1
export SLAM_LIDAR1_NAME=MID360

# Clear existing overlays before rebuilding the environment chain.
CLEANED_PREFIX=""
IFS=':'
for p in ${AMENT_PREFIX_PATH}; do
    case "$p" in
        */opt/slam/apps/*|"${NAV2_PROJECT_ROOT}"/*|*/nav2_ws/*) ;;
        *) CLEANED_PREFIX="${CLEANED_PREFIX}${CLEANED_PREFIX:+:}$p" ;;
    esac
done
unset IFS
export AMENT_PREFIX_PATH="$CLEANED_PREFIX"

# Source SLAM dependencies first, then overlay the root workspace on top so
# lumos_nav resolves to this workspace while fast_lio_* stays available.
_nav2_source_setup /opt/slam/apps/install || return 1
_nav2_source_setup "${NAV2_PROJECT_ROOT}/install" || return 1

# Restore lumos runtime libraries needed by mos_hal and related binaries.
export LD_LIBRARY_PATH=/opt/lumos/third_party/lib:/opt/lumos/lib:${LD_LIBRARY_PATH}

echo "[3dvision1] FAST-LIO + Nav2 environment ready (ROS_DOMAIN_ID=111, same domain as chassis controller)"
echo "[3dvision1] lumos_nav from ${NAV2_PROJECT_ROOT}, fast_lio_localization from /opt/slam/apps"
echo "[3dvision1] SLAM_LIDAR1_NAME=${SLAM_LIDAR1_NAME}"
echo "[3dvision1] Exit with 'source ${NAV2_PROJECT_ROOT}/unsetup_nav2.sh'"

unset _nav2_setup_script
unset _NAV2_SETUP_SUFFIX
unset _NAV2_SCRIPT_PATH
unset NAV2_PROJECT_ROOT
unset -f _nav2_source_setup
unset -f _nav2_conflicting_slam_processes
