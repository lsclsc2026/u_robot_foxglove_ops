# 真机适配指南

## 📋 目录
1. [模拟测试验证](#模拟测试验证)
2. [真机适配步骤](#真机适配步骤)
3. [硬件接口配置](#硬件接口配置)
4. [参数调优](#参数调优)
5. [故障排查](#故障排查)

---

## 📊 模拟测试验证

### 运行完整流程测试

```bash
cd /home/jeff/mos
source install/setup.bash

# 运行完整流程测试（60秒）
./mos_coordinator/test/scripts/test_full_process.sh
```

### 测试通过标准

#### ✅ 必须满足的条件
1. **状态转换完整**
   ```
   idle → SearchEmpty → APPROACHING → POSITIONING → 
   GRASPING → TRANSPORTING → RELEASING → SearchFull
   ```

2. **话题通信正常**
   - `/goal_pose` - 导航目标发布
   - `/cart_fullness/route_signal` - 装载检测结果
   - `/vision/grasp_tasks` - 把手位置
   - `/coordinator/status` - 状态发布

3. **逻辑判断正确**
   - 检测到空车时停止巡航 ✅
   - 根据把手距离选择路线（对准/直接抓取）✅
   - 抓取成功后开始运输 ✅
   - 到达B点后释放小车 ✅

4. **无错误状态**
   - 最终状态不是 `error`
   - 日志中无严重错误

### 查看测试结果

```bash
# 查看完整日志
cat /tmp/full_test_coordinator.log

# 查看状态转换
grep "状态转换:" /tmp/full_test_coordinator.log

# 查看关键事件
grep -E "(检测到|成功|失败)" /tmp/full_test_coordinator.log

# 检查最终状态
ros2 topic echo /coordinator/status --once
```

---

## 🔧 真机适配步骤

### 第一步：确认硬件系统运行

#### 1.1 导航系统 (lumos_nav)

**检查是否运行**:
```bash
ros2 node list | grep lumos_nav
```

**启动导航**:
```bash
# 根据您的实际启动方式
ros2 launch lumos_nav navigation.launch.py
```

**测试导航连接**:
```bash
# 发布测试目标
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"

# 检查是否有反馈
ros2 topic echo /navigation/feedback
```

#### 1.2 装载检测系统 (cart_fullness)

**检查节点**:
```bash
ros2 node list | grep cart_fullness
```

**启动装载检测**:
```bash
ros2 run mos_cart_fullness cart_fullness_node
```

**测试装载检测**:
```bash
# 发送检测请求
ros2 topic pub --once /cart_fullness/set_waypoint std_msgs/msg/String "data: 'left_start'"

# 监听检测结果
ros2 topic echo /cart_fullness/route_signal
```

#### 1.3 视觉抓取系统 (grasp_selector)

**检查节点**:
```bash
ros2 node list | grep grasp
```

**测试把手检测**:
```bash
# 监听把手位置
ros2 topic echo /vision/grasp_tasks
```

#### 1.4 机械臂系统 (arm_controller)

**检查服务**:
```bash
ros2 service list | grep arm
```

**测试机械臂服务**:
```bash
# 测试抓取服务是否可用
ros2 service type /mos_arm_controller/run_task
```

#### 1.5 底盘对准系统 (chassis_controller)

**检查服务**:
```bash
ros2 service list | grep chassis
```

**测试底盘服务**:
```bash
ros2 service type /mos_chassis_controller/run_chassis
```

---

### 第二步：配置真机参数

#### 2.1 修改坐标点配置

**编辑**: `coordinator_node.py`

```python
# 找到这些坐标配置，修改为真实坐标
self.patrol_points_AB = {
    'A': self._create_pose(1.0, 1.0, 0.0),  # 修改为实际A点坐标
    'B': self._create_pose(2.0, 1.0, 0.0),  # 修改为实际B点坐标
}

self.patrol_points_DE = {
    'D': self._create_pose(3.0, 1.0, 0.0),  # 修改为实际D点坐标
    'E': self._create_pose(5.0, 1.0, 0.0),  # 修改为实际E点坐标
}
```

**推荐做法**：创建配置文件
```yaml
# config/waypoints.yaml
waypoints:
  A: {x: 1.0, y: 1.0, yaw: 0.0}
  B: {x: 2.0, y: 1.0, yaw: 0.0}
  D: {x: 3.0, y: 1.0, yaw: 0.0}
  E: {x: 5.0, y: 1.0, yaw: 0.0}
```

#### 2.2 调整对准阈值

```python
# 根据实际机械臂抓取范围调整
self.alignment_threshold = 0.85  # 默认0.85米

# 如果机械臂抓取范围较大，可以增大此值
self.alignment_threshold = 1.0  # 1米内可直接抓取

# 如果需要更精确对准
self.alignment_threshold = 0.6  # 0.6米内才直接抓取
```

#### 2.3 调整重试次数

```python
# 根据实际可靠性调整
self.max_grasp_retries = 3      # 抓取重试次数
self.max_release_retries = 3    # 释放重试次数
self.max_approach_retries = 3   # 接近重试次数
```

---

### 第三步：移除模拟节点

#### 3.1 禁用dummy节点

**不要启动这些节点**:
- ❌ `dummy_cart_fullness_node`
- ❌ `dummy_grasp_selector_node`

**使用真实节点**:
- ✅ `cart_fullness_node` (真实装载检测)
- ✅ `grasp_selector_node` (真实把手检测)

#### 3.2 修改启动脚本

创建真机启动脚本：
```bash
#!/bin/bash
# launch_real_system.sh

# 启动导航
ros2 launch lumos_nav navigation.launch.py &

# 启动装载检测
ros2 run mos_cart_fullness cart_fullness_node &

# 启动把手检测
ros2 run mos_grasp_selector grasp_selector_node &

# 启动机械臂控制
ros2 run mos_arm_controller arm_controller_node &

# 启动底盘控制
ros2 run mos_chassis_controller chassis_controller_node &

# 等待所有节点就绪
sleep 5

# 启动协调器
ros2 run mos_coordinator coordinator_node
```

---

### 第四步：逐步验证

#### 4.1 静态测试（不运行完整流程）

```bash
# 只启动coordinator，检查初始化
ros2 run mos_coordinator coordinator_node

# 查看是否成功连接所有服务
# 应该看到：
# [INFO] NavigationAdapter initialized ✅
# [INFO] VisionAdapter initialized ✅
# [INFO] ArmAdapter initialized ✅
# (不应该有"服务不可用"的警告)
```

#### 4.2 分步测试

**测试1：巡航**
```bash
# 启动后应该自动开始DE巡航
# 观察机器人是否正确移动到D点和E点
```

**测试2：装载检测**
```bash
# 监听检测结果
ros2 topic echo /cart_fullness/route_signal

# 应该看到检测结果（真实的满/空判定）
```

**测试3：接近和导航**
```bash
# 检测到空车后，观察机器人是否接近小车
# 监听导航目标
ros2 topic echo /goal_pose
```

**测试4：把手检测**
```bash
# 到达小车附近后，观察是否检测到把手
ros2 topic echo /vision/grasp_tasks
```

**测试5：抓取**
```bash
# 观察机械臂是否执行抓取动作
# 检查日志：应该看到"抓取成功"
```

**测试6：运输**
```bash
# 观察机器人是否持续抓取并移动到B点
# 检查状态
ros2 topic echo /coordinator/status
# 应该显示 "transporting"
```

**测试7：释放**
```bash
# 到达B点后，观察机械臂是否释放小车
# 检查日志：应该看到"释放成功"
```

---

## 🔌 硬件接口配置

### 导航系统 (lumos_nav)

**发布的话题**:
```
/goal_pose (geometry_msgs/PoseStamped)  ← coordinator发布
/localization (geometry_msgs/PoseStamped) → coordinator订阅
```

**需要提供的反馈** (如果支持):
```
/navigation/status
/navigation/reached (到达目标点的信号)
```

### 装载检测系统

**话题接口**:
```
/cart_fullness/set_waypoint (std_msgs/String)  ← coordinator发布
/cart_fullness/route_signal (std_msgs/String)  → coordinator订阅
```

**消息格式**:
```json
{
  "decision": "full" / "not_full",
  "occupancy_ratio": 0.68,
  "used_item_count": 7
}
```

### 把手检测系统

**话题接口**:
```
/vision/grasp_tasks (自定义消息类型)  → coordinator订阅
```

**需要提供**:
- 左手把手位置 (PointStamped)
- 右手把手位置 (PointStamped)

### 机械臂系统

**服务接口**:
```
/mos_arm_controller/run_task (RunTask服务)  ← coordinator调用
```

**服务定义**:
```python
# 请求
geometry_msgs/PointStamped left_handle
geometry_msgs/PointStamped right_handle
string task_name  # "grasp" or "release"

# 响应
bool success
string message
```

### 底盘对准系统

**服务接口**:
```
/mos_chassis_controller/run_chassis (MoveChassis服务)  ← coordinator调用
```

**服务定义**:
```python
# 请求
float64 target_x_m
bool return_to_origin

# 响应
bool success
string message
```

---

## ⚙️ 参数调优

### 导航参数

```python
# 导航超时时间（秒）
navigation_timeout = 30.0

# 到达阈值（米）
arrival_threshold = 0.1
```

### 检测参数

```python
# 装载检测超时（秒）
fullness_check_timeout = 5.0

# 把手检测超时（秒）
handle_detection_timeout = 10.0
```

### 抓取参数

```python
# 对准阈值（米）
alignment_threshold = 0.85

# 抓取超时（秒）
grasp_timeout = 15.0

# 释放超时（秒）
release_timeout = 10.0
```

### 重试参数

```python
# 各阶段最大重试次数
max_grasp_retries = 3
max_release_retries = 3
max_approach_retries = 3
max_alignment_retries = 2
```

---

## 🔍 故障排查

### 问题1：导航不响应

**症状**: 发布/goal_pose后机器人不移动

**排查**:
```bash
# 检查lumos_nav是否运行
ros2 node list | grep lumos

# 检查是否订阅/goal_pose
ros2 topic info /goal_pose

# 手动发布测试
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

**解决**:
- 确认lumos_nav已启动
- 确认frame_id正确（通常是'map'）
- 检查导航系统配置

### 问题2：装载检测无响应

**症状**: 发送检测请求后无结果

**排查**:
```bash
# 检查cart_fullness节点
ros2 node list | grep cart_fullness

# 检查话题连接
ros2 topic info /cart_fullness/route_signal

# 查看日志
ros2 topic echo /cart_fullness/status
```

**解决**:
- 确认cart_fullness_node已启动
- 确认摄像头正常工作
- 检查检测模型是否加载

### 问题3：把手检测失败

**症状**: POSITIONING状态下一直等待

**排查**:
```bash
# 检查视觉系统
ros2 topic echo /vision/grasp_tasks

# 检查相机
ros2 topic list | grep camera
```

**解决**:
- 确认grasp_selector_node已启动
- 确认相机视野中有小车
- 调整光照条件

### 问题4：抓取失败

**症状**: 机械臂无法抓住小车

**排查**:
- 检查把手位置是否准确
- 检查机械臂是否正确移动
- 检查抓手状态

**解决**:
- 调整alignment_threshold
- 增加对准步骤
- 检查机械臂标定

### 问题5：状态卡住

**症状**: 某个状态下长时间无变化

**排查**:
```bash
# 查看当前状态
ros2 topic echo /coordinator/status --once

# 查看日志
cat /tmp/full_test_coordinator.log | tail -50
```

**解决**:
- 添加超时机制
- 检查是否缺少反馈信号
- 手动触发下一步（调试用）

---

## 📋 真机测试检查清单

### 启动前检查
- [ ] 所有硬件系统已上电
- [ ] 所有ROS节点已启动
- [ ] 话题连接正常
- [ ] 服务可用
- [ ] 坐标点配置正确

### 运行中监控
- [ ] 状态转换正常
- [ ] 导航目标正确
- [ ] 检测结果可靠
- [ ] 机械臂动作正确
- [ ] 无碰撞和安全问题

### 完成后验证
- [ ] 空车成功放置到B点
- [ ] 进入SearchFull状态
- [ ] 无错误日志
- [ ] 系统可以重复执行

---

## 🚀 快速开始真机测试

```bash
# 1. 编译最新代码
cd /home/jeff/mos
colcon build --packages-select mos_coordinator
source install/setup.bash

# 2. 启动所有硬件系统（根据实际情况调整）
./scripts/launch_real_system.sh

# 3. 启动协调器
ros2 run mos_coordinator coordinator_node

# 4. 观察日志和状态
ros2 topic echo /coordinator/status
```

---

**最后更新**: 2026-07-17  
**适用版本**: mos_coordinator v1.0
