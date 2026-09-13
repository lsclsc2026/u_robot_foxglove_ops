#!/bin/bash
# 第四步测试：把手检测、判断距离、对准并抓取

echo "======================================"
echo "第四步测试：把手检测与抓取"
echo "======================================"
echo ""

# Source ROS2 环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 清理之前的进程
pkill -9 -f dummy_cart_fullness 2>/dev/null
pkill -9 -f coordinator_node 2>/dev/null
pkill -9 -f dummy_grasp_selector 2>/dev/null
sleep 1

echo "启动测试环境..."
echo ""

# 启动 dummy_cart_fullness 节点
echo "1. 启动 dummy_cart_fullness 节点..."
ros2 run mos_coordinator dummy_cart_fullness_node > /tmp/step4_fullness.log 2>&1 &
FULLNESS_PID=$!
sleep 2

# 启动 dummy_grasp_selector 节点（模拟把手检测）
echo "2. 启动 dummy_grasp_selector 节点..."
ros2 run mos_coordinator dummy_grasp_selector_node > /tmp/step4_grasp_selector.log 2>&1 &
GRASP_PID=$!
sleep 2

# 启动 coordinator_node
echo "3. 启动 coordinator_node..."
ros2 run mos_coordinator coordinator_node 2>&1 | tee /tmp/step4_coordinator.log &
COORD_PID=$!
sleep 5

echo ""
echo "======================================"
echo "实时监控：完整流程 (30秒)"
echo "======================================"
echo ""

echo "预期流程："
echo "  1. SearchEmpty → 检测到空车"
echo "  2. APPROACHING → 接近小车"
echo "  3. POSITIONING → 把手检测"
echo "  4. 判断距离："
echo "     - 距离 > 0.85m → ALIGNING (对准)"
echo "     - 距离 ≤ 0.85m → GRASPING (直接抓取)"
echo "  5. GRASPING → 抓取成功/失败"
echo ""

# 实时监控状态
echo "实时状态监控："
for i in {1..30}; do
    STATE=$(timeout 1 ros2 topic echo /coordinator/status --once 2>/dev/null | grep "data:" | cut -d: -f2 | tr -d ' ')
    if [ -n "$STATE" ]; then
        echo "  [${i}s] 当前状态: $STATE"
    fi
    sleep 1
done

echo ""
echo "======================================"
echo "检查最终状态和关键话题"
echo "======================================"
echo ""

echo "1. 最终状态："
timeout 2 ros2 topic echo /coordinator/status --once
echo ""

echo "2. 检查 /vision/grasp_tasks 话题（把手位置）："
ros2 topic info /vision/grasp_tasks
echo ""

echo "3. 尝试读取把手检测消息："
timeout 3 ros2 topic echo /vision/grasp_tasks --once 2>&1 || echo "[未检测到消息]"
echo ""

echo "======================================"
echo "完整日志分析"
echo "======================================"
echo ""

echo "✓ coordinator关键日志："
grep -E "(SearchEmpty|APPROACHING|POSITIONING|把手检测|距离|对准|GRASPING|抓取)" /tmp/step4_coordinator.log | tail -30
echo ""

echo "✓ grasp_selector日志："
cat /tmp/step4_grasp_selector.log | tail -20
echo ""

echo "======================================"
echo "话题统计"
echo "======================================"
echo ""

echo "所有相关话题："
ros2 topic list | grep -E "(vision|grasp|coordinator|arm)"
echo ""

echo "======================================"
echo "清理环境"
echo "======================================"
echo ""

kill $FULLNESS_PID 2>/dev/null
kill $GRASP_PID 2>/dev/null
kill $COORD_PID 2>/dev/null
sleep 2

pkill -9 -f dummy_cart_fullness 2>/dev/null
pkill -9 -f dummy_grasp_selector 2>/dev/null
pkill -9 -f coordinator_node 2>/dev/null

echo ""
echo "✅ 第四步测试完成！"
echo ""
echo "详细日志文件："
echo "  - /tmp/step4_coordinator.log"
echo "  - /tmp/step4_fullness.log"
echo "  - /tmp/step4_grasp_selector.log"
echo ""
echo "验证要点："
echo "  ✓ POSITIONING状态下收到把手位置"
echo "  ✓ 根据距离判断是否需要对准"
echo "  ✓ 距离合适直接抓取 or 对准后抓取"
echo "  ✓ 抓取失败时重试机制"
echo ""
