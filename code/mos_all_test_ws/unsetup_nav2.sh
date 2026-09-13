#!/bin/bash
# ============================================================
# 恢复 3dvision1 Nav2 之前的环境
# ============================================================

if [ -n "${_NAV2_SAVED_AMENT_PREFIX}" ]; then
    export AMENT_PREFIX_PATH="${_NAV2_SAVED_AMENT_PREFIX}"
    unset _NAV2_SAVED_AMENT_PREFIX
fi

if [ -n "${_NAV2_SAVED_ROS_DOMAIN}" ]; then
    export ROS_DOMAIN_ID="${_NAV2_SAVED_ROS_DOMAIN}"
    unset _NAV2_SAVED_ROS_DOMAIN
else
    unset ROS_DOMAIN_ID
fi

if [ -n "${_NAV2_SAVED_ROS_LOCALHOST}" ]; then
    export ROS_LOCALHOST_ONLY="${_NAV2_SAVED_ROS_LOCALHOST}"
    unset _NAV2_SAVED_ROS_LOCALHOST
else
    unset ROS_LOCALHOST_ONLY
fi

if [ -n "${_NAV2_SAVED_LD_LIBRARY}" ]; then
    export LD_LIBRARY_PATH="${_NAV2_SAVED_LD_LIBRARY}"
    unset _NAV2_SAVED_LD_LIBRARY
fi

if [ -n "${_NAV2_SAVED_PATH}" ]; then
    export PATH="${_NAV2_SAVED_PATH}"
    unset _NAV2_SAVED_PATH
fi

if [ -n "${_NAV2_SAVED_PYTHONPATH}" ]; then
    export PYTHONPATH="${_NAV2_SAVED_PYTHONPATH}"
    unset _NAV2_SAVED_PYTHONPATH
else
    unset PYTHONPATH
fi

if [ -n "${_NAV2_SAVED_CMAKE_PREFIX}" ]; then
    export CMAKE_PREFIX_PATH="${_NAV2_SAVED_CMAKE_PREFIX}"
    unset _NAV2_SAVED_CMAKE_PREFIX
else
    unset CMAKE_PREFIX_PATH
fi

if [ -n "${_NAV2_SAVED_COLCON_PREFIX}" ]; then
    export COLCON_PREFIX_PATH="${_NAV2_SAVED_COLCON_PREFIX}"
    unset _NAV2_SAVED_COLCON_PREFIX
else
    unset COLCON_PREFIX_PATH
fi

echo "[3dvision1] 环境已恢复"
