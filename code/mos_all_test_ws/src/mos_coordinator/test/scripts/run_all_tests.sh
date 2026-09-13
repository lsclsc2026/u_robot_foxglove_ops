#!/bin/bash
# 一键运行所有测试

echo "======================================"
echo "MOS Coordinator - 找空阶段测试套件"
echo "======================================"
echo ""

# 检查编译
echo "步骤 1/3: 检查编译状态..."
if [ ! -d "install/mos_coordinator" ]; then
    echo "❌ 未找到编译输出，正在编译..."
    colcon build --packages-select mos_coordinator
else
    echo "✅ 编译输出存在"
fi

echo ""
echo "步骤 2/3: 运行基本功能测试..."
./mos_coordinator/test/scripts/test_search_empty.sh

echo ""
echo "步骤 3/3: 运行话题频率测试..."
./mos_coordinator/test/scripts/test_topics_hz.sh

echo ""
echo "======================================"
echo "✅ 所有测试完成！"
echo "======================================"
echo ""
echo "查看详细文档："
echo "  - 测试说明: mos_coordinator/test/scripts/README.md"
echo "  - 实现总结: mos_coordinator/test/scripts/STEP1_SUMMARY.md"
echo ""
