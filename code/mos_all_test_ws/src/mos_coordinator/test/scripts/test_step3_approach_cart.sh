#!/bin/bash
# 第三步测试：接近空车并导航

echo "======================================"
echo "第三步测试：接近空车并导航"
echo "======================================"
echo ""

# Source ROS2 环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 清理之前的进程
pkill -9 -f dummy_cart_fullness 2>/dev/null
pkill -9 -f coordinator_node 2>/dev/null
sleep 1

echo "启动测试环境..."
echo ""

# 启动 dummy_cart_fullness 节点
echo "1. 启动 dummy_cart_fullness 节点..."
ros2 run mos_coordinator dummy_cart_fullness_node > /tmp/step3_fullness.log 2>&1 &
FULLNESS_PID=$!
echo "   PID: $FULLNESS_PID"
sleep 2

# 启动 coordinator_node
echo "2. 启动 coordinator_node..."
ros2 run mos_coordinator coordinator_node > /tmp/step3_coordinator.log 2>&1 &
COORD_PID=$!
echo "   PID: $COORD_PID"
sleep 5

echo ""
echo "======================================"
echo "第一阶段：等待检测到空车 (8秒)"
echo "======================================"
echo ""

echo "监控流程："
echo "  1. SearchEmpty → 巡航DE区间"
echo "  2. 装载检测 → 发现空车"
echo "  3. 停止巡航 → 转APPROACHING状态"
echo ""

sleep 8

echo "======================================"
echo "第二阶段：检查状态转换"
echo "======================================"
echo ""

echo "当前状态："
timeout 2 ros2 topic echo /coordinator/status --once
echo ""

echo "======================================"
echo "第三阶段：验证/goal_pose发布"
echo "======================================"
echo ""

echo "检查导航目标（接近小车位置）："
timeout 3 ros2 topic echo /goal_pose --once
echo ""

echo "======================================"
echo "第四阶段：话题频率测试"
echo "======================================"
echo ""

echo "1. /goal_pose 发布频率："
timeout 5 ros2 topic hz /goal_pose 2>&1 || echo "   [未检测到新发布]"
echo ""

echo "2. /coordinator/status 发布频率："
timeout 5 ros2 topic hz /coordinator/status 2>&1 || echo "   [未检测到发布]"
echo ""

echo "======================================"
echo "第五阶段：检查日志"
echo "======================================"
echo ""

echo "✓ coordinator 关键日志："
grep -E "(检测到空载|接近阶段|到达小车|APPROACHING|POSITIONING)" /tmp/step3_coordinator.log | tail -15
echo ""

echo "✓ dummy_cart_fullness 关键日志："
grep -E "(判定结果|decision)" /tmp/step3_fullness.log | tail -5
echo ""

echo "======================================"
echo "第六阶段：话题信息验证"
echo "======================================"
echo ""

echo "✓ /goal_pose 话题信息："
ros2 topic info /goal_pose
echo ""

echo "======================================"
echo "清理环境"
echo "======================================"
echo ""

kill $FULLNESS_PID 2>/dev/null
kill $COORD_PID 2>/dev/null
sleep 2

pkill -9 -f dummy_cart_fullness 2>/dev/null
pkill -9 -f coordinator_node 2>/dev/null

echo "✅ 第三步测试完成！"
echo ""
echo "测试报告："
echo "  日志文件："
echo "    - /tmp/step3_coordinator.log"
echo "    - /tmp/step3_fullness.log"
echo ""
echo "验证要点："
echo "  ✓ 检测到空车后停止巡航"
echo "  ✓ 状态转换: SearchEmpty → APPROACHING"
echo "  ✓ 发布 /goal_pose 导航目标"
echo "  ✓ 目标位置为小车当前位置"
echo ""
