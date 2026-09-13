#!/bin/bash
# ============================================================
# 3dvision1 Nav2 环境隔离脚本
# 使用前: source /root/3dvision1/setup_nav2.sh
# 使用后: source /root/3dvision1/unsetup_nav2.sh (恢复环境)
# ============================================================
# ROS_DOMAIN_ID=109 与 mos_slam 系统(108)隔离，DDS层面互不干扰

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

# 保存当前环境
export _NAV2_SAVED_AMENT_PREFIX="${AMENT_PREFIX_PATH}"
export _NAV2_SAVED_ROS_DOMAIN="${ROS_DOMAIN_ID}"
export _NAV2_SAVED_ROS_LOCALHOST="${ROS_LOCALHOST_ONLY}"
export _NAV2_SAVED_LD_LIBRARY="${LD_LIBRARY_PATH}"
export _NAV2_SAVED_PATH="${PATH}"
export _NAV2_SAVED_PYTHONPATH="${PYTHONPATH}"
export _NAV2_SAVED_CMAKE_PREFIX="${CMAKE_PREFIX_PATH}"
export _NAV2_SAVED_COLCON_PREFIX="${COLCON_PREFIX_PATH}"

# 与底盘控制/键盘遥控统一的 ROS_DOMAIN_ID
export ROS_DOMAIN_ID=111
export ROS_LOCALHOST_ONLY=1

# 清理 AMENT_PREFIX_PATH 中的 slam workspace 路径
CLEANED_PREFIX=""
IFS=':'
for p in ${AMENT_PREFIX_PATH}; do
    case "$p" in
        */opt/slam/apps/*) ;;
        *) CLEANED_PREFIX="${CLEANED_PREFIX}${CLEANED_PREFIX:+:}$p" ;;
    esac
done
unset IFS
export AMENT_PREFIX_PATH="$CLEANED_PREFIX"

# 使用根目录唯一工作空间的 overlay 环境
_nav2_source_setup "${NAV2_PROJECT_ROOT}/install" || return 1

# 恢复 LD_LIBRARY_PATH 中的 lumos 依赖
export LD_LIBRARY_PATH=/opt/lumos/third_party/lib:/opt/lumos/lib:${LD_LIBRARY_PATH}

echo "[3dvision1] Nav2 环境就绪 (ROS_DOMAIN_ID=111, 与底盘控制同域)"
echo "[3dvision1] 退出后运行 'source ${NAV2_PROJECT_ROOT}/unsetup_nav2.sh' 恢复环境"

unset _nav2_setup_script
unset _NAV2_SETUP_SUFFIX
unset _NAV2_SCRIPT_PATH
unset NAV2_PROJECT_ROOT
unset -f _nav2_source_setup
