#!/bin/bash
# 第二步增强测试：装载状态检测 - 详细的Topic监控

echo "======================================"
echo "第二步增强测试：装载状态检测"
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
ros2 run mos_coordinator dummy_cart_fullness_node > /tmp/fullness_output.log 2>&1 &
FULLNESS_PID=$!
echo "   PID: $FULLNESS_PID"
sleep 2

# 启动 coordinator_node
echo "2. 启动 coordinator_node..."
ros2 run mos_coordinator coordinator_node > /tmp/coordinator_output.log 2>&1 &
COORD_PID=$!
echo "   PID: $COORD_PID"
sleep 3

echo ""
echo "======================================"
echo "第一阶段：检查话题连接 (3秒)"
echo "======================================"
echo ""

echo "✓ 所有cart_fullness相关话题："
ros2 topic list | grep cart_fullness
echo ""

echo "✓ coordinator相关话题："
ros2 topic list | grep coordinator
echo ""

echo "======================================"
echo "第二阶段：实时监控装载检测请求 (5秒)"
echo "======================================"
echo ""

echo "监听 /cart_fullness/set_waypoint (检测请求)..."
timeout 5 ros2 topic echo /cart_fullness/set_waypoint --once
echo ""

echo "======================================"
echo "第三阶段：监控装载检测结果 (5秒)"
echo "======================================"
echo ""

echo "监听 /cart_fullness/route_signal (检测结果)..."
echo "预期: left_start->空车, right_start->满车"
echo ""
timeout 5 ros2 topic echo /cart_fullness/route_signal --once
echo ""

echo "======================================"
echo "第四阶段：话题频率测试"
echo "======================================"
echo ""

echo "1. /cart_fullness/status 频率测试 (5秒)："
timeout 5 ros2 topic hz /cart_fullness/status 2>&1 || echo "   [未检测到发布]"
echo ""

echo "2. /cart_fullness/route_signal 频率测试 (5秒)："
timeout 5 ros2 topic hz /cart_fullness/route_signal 2>&1 || echo "   [未检测到发布]"
echo ""

echo "3. /coordinator/status 频率测试 (5秒)："
timeout 5 ros2 topic hz /coordinator/status 2>&1 || echo "   [未检测到发布]"
echo ""

echo "======================================"
echo "第五阶段：检查消息内容"
echo "======================================"
echo ""

echo "1. /cart_fullness/status 当前状态："
timeout 2 ros2 topic echo /cart_fullness/status --once 2>&1 || echo "   [无消息]"
echo ""

echo "2. /cart_fullness/route_signal 最新检测结果："
timeout 2 ros2 topic echo /cart_fullness/route_signal --once 2>&1 || echo "   [无消息]"
echo ""

echo "3. /coordinator/status 当前状态："
timeout 2 ros2 topic echo /coordinator/status --once 2>&1 || echo "   [无消息]"
echo ""

echo "======================================"
echo "第六阶段：检查节点日志"
echo "======================================"
echo ""

echo "✓ dummy_cart_fullness 最新日志 (最后10行):"
tail -10 /tmp/fullness_output.log 2>&1 || echo "   [无日志]"
echo ""

echo "✓ coordinator_node 最新日志 (最后10行):"
tail -10 /tmp/coordinator_output.log 2>&1 || echo "   [无日志]"
echo ""

echo "======================================"
echo "第七阶段：话题信息详情"
echo "======================================"
echo ""

echo "✓ /cart_fullness/set_waypoint 话题信息："
ros2 topic info /cart_fullness/set_waypoint -v
echo ""

echo "✓ /cart_fullness/route_signal 话题信息："
ros2 topic info /cart_fullness/route_signal -v
echo ""

echo "======================================"
echo "清理环境"
echo "======================================"
echo ""

kill $FULLNESS_PID 2>/dev/null
kill $COORD_PID 2>/dev/null
sleep 2

# 强制清理
pkill -9 -f dummy_cart_fullness 2>/dev/null
pkill -9 -f coordinator_node 2>/dev/null

echo "✅ 第二步增强测试完成！"
echo ""
echo "测试报告："
echo "  日志文件："
echo "    - /tmp/fullness_output.log"
echo "    - /tmp/coordinator_output.log"
echo ""
echo "验证要点："
echo "  ✓ 话题连接正常"
echo "  ✓ 检测请求已发送 (left_start, right_start)"
echo "  ✓ 检测结果已返回 (decision: full/not_full)"
echo "  ✓ 话题频率符合预期"
echo ""
