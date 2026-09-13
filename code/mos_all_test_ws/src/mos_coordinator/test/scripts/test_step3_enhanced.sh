#!/bin/bash
# 第三步增强测试：接近空车并导航（等待完整流程）

echo "======================================"
echo "第三步增强测试：接近空车并导航"
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
ros2 run mos_coordinator dummy_cart_fullness_node > /tmp/step3_fullness_v2.log 2>&1 &
FULLNESS_PID=$!
echo "   PID: $FULLNESS_PID"
sleep 2

# 启动 coordinator_node
echo "2. 启动 coordinator_node..."
ros2 run mos_coordinator coordinator_node 2>&1 | tee /tmp/step3_coordinator_v2.log &
COORD_PID=$!
echo "   PID: $COORD_PID"
sleep 5

echo ""
echo "======================================"
echo "实时监控：等待完整流程 (20秒)"
echo "======================================"
echo ""

echo "预期流程："
echo "  1. SearchEmpty → 巡航并发送检测请求"
echo "  2. dummy_cart_fullness → 检测并返回空车结果"
echo "  3. coordinator → 停止巡航，转APPROACHING"
echo "  4. coordinator → 发布/goal_pose导航到小车"
echo ""

# 实时监控coordinator状态
echo "实时状态监控："
for i in {1..20}; do
    STATE=$(timeout 1 ros2 topic echo /coordinator/status --once 2>/dev/null | grep "data:" | cut -d: -f2 | tr -d ' ')
    if [ -n "$STATE" ]; then
        echo "  [${i}s] 当前状态: $STATE"
    fi
    sleep 1
done

echo ""
echo "======================================"
echo "检查最终状态"
echo "======================================"
echo ""

echo "最终状态："
timeout 2 ros2 topic echo /coordinator/status --once
echo ""

echo "======================================"
echo "检查导航目标"
echo "======================================"
echo ""

echo "检查是否发布了/goal_pose："
ros2 topic info /goal_pose -v
echo ""

echo "尝试读取最新的goal_pose："
timeout 3 ros2 topic echo /goal_pose --once 2>&1 || echo "[未检测到goal_pose消息]"
echo ""

echo "======================================"
echo "完整日志分析"
echo "======================================"
echo ""

echo "✓ coordinator完整流程日志："
grep -E "(SearchEmpty|装载检测|空载小车|APPROACHING|接近|goal_pose)" /tmp/step3_coordinator_v2.log
echo ""

echo "✓ dummy_cart_fullness完整流程日志："
cat /tmp/step3_fullness_v2.log | grep -v "^$"
echo ""

echo "======================================"
echo "话题统计"
echo "======================================"
echo ""

echo "所有相关话题："
ros2 topic list | grep -E "(goal_pose|coordinator|cart_fullness)"
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

echo ""
echo "✅ 第三步增强测试完成！"
echo ""
echo "详细日志文件："
echo "  - /tmp/step3_coordinator_v2.log"
echo "  - /tmp/step3_fullness_v2.log"
echo ""
