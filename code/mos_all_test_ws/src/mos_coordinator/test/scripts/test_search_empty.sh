#!/bin/bash
# 测试找空阶段的脚本

echo "======================================"
echo "测试找空阶段 - 巡航和装载检测"
echo "======================================"

# Source ROS2 环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo ""
echo "启动 coordinator_node..."
echo ""

# 启动节点（后台运行）
ros2 run mos_coordinator coordinator_node &
COORD_PID=$!

echo "Coordinator PID: $COORD_PID"
echo ""
echo "等待节点初始化 (5秒)..."
sleep 5

echo ""
echo "======================================"
echo "检查关键话题"
echo "======================================"
echo ""

# 检查状态话题
echo "1. 检查 /coordinator/status 话题："
timeout 3 ros2 topic echo /coordinator/status --once

echo ""
echo "2. 检查 /coordinator/status 发布频率："
timeout 5 ros2 topic hz /coordinator/status

echo ""
echo "3. 检查 /goal_pose 话题（导航目标）："
timeout 3 ros2 topic echo /goal_pose --once

echo ""
echo "4. 检查 /cart_fullness/set_waypoint 话题（装载检测请求）："
timeout 3 ros2 topic echo /cart_fullness/set_waypoint --once

echo ""
echo "======================================"
echo "话题列表"
echo "======================================"
ros2 topic list | grep -E "(coordinator|goal_pose|cart_fullness)"

echo ""
echo "======================================"
echo "停止测试"
echo "======================================"
kill $COORD_PID 2>/dev/null
echo "测试完成！"
