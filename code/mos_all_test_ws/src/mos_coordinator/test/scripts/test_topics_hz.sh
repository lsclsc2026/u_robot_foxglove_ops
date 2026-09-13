#!/bin/bash
# 详细测试找空阶段的话题频率

echo "======================================"
echo "测试找空阶段 - 话题频率测试"
echo "======================================"

# Source ROS2 环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo ""
echo "启动 coordinator_node..."
ros2 run mos_coordinator coordinator_node &
COORD_PID=$!

echo "Coordinator PID: $COORD_PID"
echo ""
echo "等待节点初始化 (3秒)..."
sleep 3

echo ""
echo "======================================"
echo "话题频率测试 (Hz)"
echo "======================================"
echo ""

echo "1. /coordinator/status (状态发布) - 预期: 1 Hz"
timeout 5 ros2 topic hz /coordinator/status
echo ""

echo "2. /goal_pose (导航目标发布) - 检查是否发布"
timeout 5 ros2 topic hz /goal_pose
echo ""

echo "3. /cart_fullness/set_waypoint (装载检测请求) - 检查是否发布"
timeout 3 ros2 topic echo /cart_fullness/set_waypoint
echo ""

echo "======================================"
echo "话题内容抽样"
echo "======================================"
echo ""

echo "4. 查看 /coordinator/status 当前状态："
timeout 2 ros2 topic echo /coordinator/status --once
echo ""

echo "5. 查看 /goal_pose 当前导航目标："
timeout 2 ros2 topic echo /goal_pose --once
echo ""

echo "======================================"
echo "所有相关话题列表"
echo "======================================"
ros2 topic list | grep -E "(coordinator|goal|cart_fullness|localization)"

echo ""
echo "======================================"
echo "停止测试"
echo "======================================"
kill $COORD_PID 2>/dev/null
sleep 1
echo "✅ 测试完成！"
