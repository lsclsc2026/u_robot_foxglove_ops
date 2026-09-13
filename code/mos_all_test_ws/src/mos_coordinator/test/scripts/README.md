# MOS Coordinator 测试脚本

本目录包含用于测试 MOS Coordinator 各个功能阶段的测试脚本。

## 测试脚本列表

### 1. test_search_empty.sh
**功能**: 测试找空阶段的基本功能

**测试内容**:
- 启动 coordinator_node
- 检查关键话题是否正常发布
- 验证状态机是否正确进入 SearchEmpty 状态
- 检查装载检测请求是否发出

**使用方法**:
```bash
cd /home/jeff/mos
source install/setup.bash
./mos_coordinator/test/scripts/test_search_empty.sh
```

**预期输出**:
- `/coordinator/status` 话题输出 "SearchEmpty"
- `/goal_pose` 话题存在并发布导航目标
- `/cart_fullness/set_waypoint` 话题发布检测请求

---

### 2. test_topics_hz.sh
**功能**: 测试话题发布频率 (Hz)

**测试内容**:
- `/coordinator/status` 发布频率（预期 ~1-2 Hz）
- `/goal_pose` 发布情况
- `/cart_fullness/set_waypoint` 内容检查
- 话题内容抽样查看

**使用方法**:
```bash
cd /home/jeff/mos
source install/setup.bash
./mos_coordinator/test/scripts/test_topics_hz.sh
```

**预期输出**:
- `/coordinator/status`: 约 1-2 Hz
- 显示各话题的当前内容
- 列出所有相关话题

---

## 测试前准备

1. **编译功能包**:
```bash
cd /home/jeff/mos
colcon build --packages-select mos_coordinator
source install/setup.bash
```

2. **检查依赖**:
确保以下 ROS2 功能包已安装或可用：
- mos_3d_object (可选)
- mos_grasp_selector (可选)
- mos_arm_controller (可选)
- mos_chassis_controller (可选)
- cart_fullness (装载检测，必需)

## 测试结果说明

### 成功标志
- ✅ 节点正常启动，无报错退出
- ✅ `/coordinator/status` 输出 "SearchEmpty"
- ✅ `/goal_pose` 话题存在并发布巡航点
- ✅ `/cart_fullness/set_waypoint` 发布 "left_start" 和 "right_start"

### 警告信息（可忽略）
以下警告信息不影响测试：
- `MoveChassis 服务不可用` - 底盘对准功能暂时禁用
- `DetectedObject3DArray 消息类型不可用` - 3D检测功能暂时禁用
- `GraspTaskArray 消息类型不可用` - 抓取任务功能暂时禁用
- `RunTask 服务不可用` - 机械臂功能暂时禁用
- `手臂初始化失败` - 测试环境无手臂硬件

## 故障排查

### 问题: 节点无法启动
**解决方案**:
1. 检查是否已编译: `colcon build --packages-select mos_coordinator`
2. 检查是否已 source: `source install/setup.bash`
3. 检查 ROS2 环境: `ros2 topic list`

### 问题: 话题不存在
**解决方案**:
1. 等待节点完全初始化（约3-5秒）
2. 检查节点是否正常运行: `ros2 node list`
3. 查看节点日志输出

### 问题: 状态不是 SearchEmpty
**解决方案**:
检查 `coordinator_node.py` 中的 `start_task()` 方法是否正确调用

## 开发说明

### 添加新测试脚本
1. 在本目录创建新的 `.sh` 文件
2. 添加执行权限: `chmod +x test_xxx.sh`
3. 更新本 README 文档

### 测试模板
```bash
#!/bin/bash
# 测试描述

echo "======================================"
echo "测试名称"
echo "======================================"

# Source ROS2 环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动节点
ros2 run mos_coordinator coordinator_node &
NODE_PID=$!

# 等待初始化
sleep 3

# 执行测试
# ...

# 清理
kill $NODE_PID 2>/dev/null
echo "测试完成！"
```

---

**最后更新**: 2026-07-17  
**维护者**: MOS Team
