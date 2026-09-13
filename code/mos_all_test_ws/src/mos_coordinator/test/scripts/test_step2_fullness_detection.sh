#!/bin/bash
# 第二步测试：装载状态检测 - Topic信号和Hz验证

echo "======================================"
echo "第二步测试：装载状态检测"
echo "======================================"
echo ""

# Source ROS2 环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "启动节点..."
echo ""

# 启动 dummy_cart_fullness 节点（后台运行）
echo "1. 启动 dummy_cart_fullness 节点..."
ros2 run mos_coordinator dummy_cart_fullness_node &
FULLNESS_PID=$!
sleep 2

# 启动 coordinator_node（后台运行）
echo "2. 启动 coordinator_node..."
ros2 run mos_coordinator coordinator_node &
COORD_PID=$!
sleep 3

echo ""
echo "======================================"
echo "检查话题连接状态"
echo "======================================"
echo ""

echo "所有相关话题："
ros2 topic list | grep -E "(cart_fullness|coordinator)"
echo ""

echo "======================================"
echo "监控装载检测流程 (15秒)"
echo "======================================"
echo ""

echo "监听 /cart_fullness/route_signal (装载检测结果)..."
timeout 15 ros2 topic echo /cart_fullness/route_signal &
ECHO_PID=$!

echo ""
echo "等待检测过程..."
sleep 15

echo ""
echo "======================================"
echo "话题频率测试"
echo "======================================"
echo ""

echo "1. /cart_fullness/status 发布频率："
timeout 5 ros2 topic hz /cart_fullness/status
echo ""

echo "2. /cart_fullness/route_signal 发布频率："
timeout 5 ros2 topic hz /cart_fullness/route_signal
echo ""

echo "3. /coordinator/status 发布频率："
timeout 5 ros2 topic hz /coordinator/status
echo ""

echo "======================================"
echo "检查最新消息内容"
echo "======================================"
echo ""

echo "1. /cart_fullness/status 最新状态："
timeout 2 ros2 topic echo /cart_fullness/status --once
echo ""

echo "2. /cart_fullness/route_signal 最新结果："
timeout 2 ros2 topic echo /cart_fullness/route_signal --once
echo ""

echo "3. /coordinator/status 当前状态："
timeout 2 ros2 topic echo /coordinator/status --once
echo ""

echo "======================================"
echo "停止所有节点"
echo "======================================"
kill $FULLNESS_PID 2>/dev/null
kill $COORD_PID 2>/dev/null
kill $ECHO_PID 2>/dev/null
sleep 1

echo ""
echo "✅ 第二步测试完成！"
echo ""
echo "预期结果："
echo "  - /cart_fullness/set_waypoint: coordinator发送检测请求"
echo "  - /cart_fullness/route_signal: 返回装载状态 (full/not_full)"
echo "  - left_start -> not_full (空车)"
echo "  - right_start -> full (满车)"
echo ""
