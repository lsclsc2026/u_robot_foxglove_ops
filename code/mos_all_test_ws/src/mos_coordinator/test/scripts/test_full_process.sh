#!/bin/bash
# 完整流程测试：找空阶段从头到尾

echo "======================================"
echo "找空阶段完整流程测试"
echo "======================================"
echo ""

# Source ROS2 环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 清理之前的进程
echo "清理环境..."
pkill -9 -f dummy_cart_fullness 2>/dev/null
pkill -9 -f dummy_grasp_selector 2>/dev/null
pkill -9 -f coordinator_node 2>/dev/null
sleep 2

echo ""
echo "======================================"
echo "启动模拟环境"
echo "======================================"
echo ""

# 启动 dummy_cart_fullness 节点（模拟装载检测）
echo "1. 启动 dummy_cart_fullness（装载检测模拟）..."
ros2 run mos_coordinator dummy_cart_fullness_node > /tmp/full_test_fullness.log 2>&1 &
FULLNESS_PID=$!
echo "   PID: $FULLNESS_PID"
sleep 2

# 启动 dummy_grasp_selector 节点（模拟把手检测）
echo "2. 启动 dummy_grasp_selector（把手检测模拟）..."
ros2 run mos_coordinator dummy_grasp_selector_node > /tmp/full_test_grasp.log 2>&1 &
GRASP_PID=$!
echo "   PID: $GRASP_PID"
sleep 2

# 启动 coordinator_node
echo "3. 启动 coordinator_node（主协调器）..."
ros2 run mos_coordinator coordinator_node 2>&1 | tee /tmp/full_test_coordinator.log &
COORD_PID=$!
echo "   PID: $COORD_PID"
sleep 5

echo ""
echo "======================================"
echo "完整流程监控 (60秒)"
echo "======================================"
echo ""

echo "预期流程："
echo "  1️⃣  SearchEmpty → 巡航DE区间"
echo "  2️⃣  检测到空车 → APPROACHING"
echo "  3️⃣  到达小车 → POSITIONING"
echo "  4️⃣  检测把手 → 判断距离"
echo "       ├─ > 0.85m → ALIGNING"
echo "       └─ ≤ 0.85m → GRASPING"
echo "  5️⃣  GRASPING → 抓取成功"
echo "  6️⃣  TRANSPORTING → 运输到B点"
echo "  7️⃣  RELEASING → 释放小车"
echo "  8️⃣  SearchFull → 找空阶段完成 ✅"
echo ""

# 实时监控状态变化
echo "实时状态监控："
LAST_STATE=""
for i in {1..60}; do
    STATE=$(timeout 1 ros2 topic echo /coordinator/status --once 2>/dev/null | grep "data:" | cut -d: -f2 | tr -d ' ')
    if [ -n "$STATE" ] && [ "$STATE" != "$LAST_STATE" ]; then
        echo "  [${i}s] 状态变化: $LAST_STATE → $STATE"
        LAST_STATE=$STATE
    fi
    sleep 1
done

echo ""
echo "======================================"
echo "最终状态检查"
echo "======================================"
echo ""

echo "1. 最终状态："
FINAL_STATE=$(timeout 2 ros2 topic echo /coordinator/status --once 2>/dev/null | grep "data:" | cut -d: -f2 | tr -d ' ')
echo "   当前状态: $FINAL_STATE"

if [ "$FINAL_STATE" == "SearchFull" ]; then
    echo "   ✅ 找空阶段完成！已进入找满阶段"
elif [ "$FINAL_STATE" == "error" ]; then
    echo "   ❌ 流程执行出错"
else
    echo "   ⚠️  流程未完成，当前状态: $FINAL_STATE"
fi
echo ""

echo "======================================"
echo "流程分析"
echo "======================================"
echo ""

echo "✓ 关键状态转换："
grep "状态转换:" /tmp/full_test_coordinator.log | tail -20
echo ""

echo "✓ 关键事件："
grep -E "(检测到空载|接近阶段|把手检测|距离|抓取成功|运输阶段|释放)" /tmp/full_test_coordinator.log | tail -30
echo ""

echo "======================================"
echo "话题统计"
echo "======================================"
echo ""

echo "发布的关键话题："
ros2 topic list | grep -E "(coordinator|goal_pose|cart_fullness|vision|arm)" | sort
echo ""

echo "======================================"
echo "性能统计"
echo "======================================"
echo ""

# 统计各阶段耗时
echo "各阶段耗时分析："
grep "状态转换:" /tmp/full_test_coordinator.log | while read line; do
    echo "  $line"
done
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
echo "✅ 完整流程测试结束！"
echo ""
echo "详细日志文件："
echo "  - /tmp/full_test_coordinator.log"
echo "  - /tmp/full_test_fullness.log"
echo "  - /tmp/full_test_grasp.log"
echo ""
echo "查看完整日志："
echo "  cat /tmp/full_test_coordinator.log"
echo ""
