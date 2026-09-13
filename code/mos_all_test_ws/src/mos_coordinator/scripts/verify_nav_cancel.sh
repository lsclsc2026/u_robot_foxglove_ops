#!/bin/bash

echo "======================================"
echo "   导航取消功能实时验证"
echo "======================================"
echo ""

# 检查 ROS2 环境
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS2 未找到，请先 source setup"
    exit 1
fi

echo "[检查1] cmd_vel 话题..."
CMD_VEL=$(timeout 1 ros2 topic echo /cmd_vel --once 2>/dev/null | grep -A1 "linear:" | tail -1 | awk '{print $2}')
if [ -z "$CMD_VEL" ]; then
    echo "  ❌ 无法读取 /cmd_vel"
else
    echo "  ✅ 当前速度: $CMD_VEL m/s"
    if (( $(echo "$CMD_VEL < 0.01" | bc -l 2>/dev/null || echo "0") )); then
        echo "  ✅ 机器人已停止"
    else
        echo "  ⚠️ 机器人正在运动"
    fi
fi

echo ""
echo "[检查2] Action 状态..."
ACTION_CLIENTS=$(ros2 action info /navigate_to_pose 2>/dev/null | grep "Action clients" | awk '{print $3}')
if [ -z "$ACTION_CLIENTS" ]; then
    echo "  ❌ 无法读取 action 信息"
else
    echo "  ✅ Action clients: $ACTION_CLIENTS"
    if [ "$ACTION_CLIENTS" = "0" ]; then
        echo "  ✅ 没有活动的导航任务"
    else
        echo "  ⚠️ 有 $ACTION_CLIENTS 个活动的导航任务"
    fi
fi

echo ""
echo "[检查3] CancelGoal 服务..."
if ros2 service list 2>/dev/null | grep -q "cancel_goal"; then
    echo "  ✅ CancelGoal 服务可用"
else
    echo "  ❌ CancelGoal 服务不可用"
fi

echo ""
echo "[检查4] Coordinator 节点..."
if ros2 node list 2>/dev/null | grep -q "task_coordinator"; then
    echo "  ✅ task_coordinator 节点运行中"
else
    echo "  ❌ task_coordinator 节点未运行"
fi

echo ""
echo "[检查5] 定位数据..."
if timeout 1 ros2 topic echo /localization --once >/dev/null 2>&1; then
    echo "  ✅ /localization 话题有数据"
else
    echo "  ❌ /localization 话题无数据"
fi

echo ""
echo "======================================"
echo "  建议：同时打开3个终端监控"
echo "======================================"
echo ""
echo "终端1（cmd_vel）:"
echo "  watch -n 0.1 \"ros2 topic echo /cmd_vel --once | grep -A1 'linear:'\""
echo ""
echo "终端2（action状态）:"
echo "  watch -n 0.5 \"ros2 action info /navigate_to_pose | grep 'Action clients'\""
echo ""
echo "终端3（日志）:"
echo "  ros2 topic echo /rosout | grep -E \"取消|cancel|停止|超时\""
