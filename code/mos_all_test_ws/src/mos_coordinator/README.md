# MOS Coordinator - 医院巡航机器人任务协调中间件

## 📋 项目简介

MOS Coordinator 是医院巡航机器人的核心任务协调系统，负责统一调度导航、视觉、机械臂等子系统，实现小车的自动拾取、运输和卸货的完整闭环流程。
部分流程尚未核实，部分接口需要调校

### 核心功能
- ✅ 分区自动巡航
- ✅ 视觉目标检测与定位
- ✅ 智能路径规划与接近
- ✅ 底盘自动对准
- ✅ 机械臂抓取控制
- ✅ 自动运输与释放
- ✅ 完整任务闭环

---

## 🏗️ 系统架构

```
┌─────────────────────────────────────────┐
│         MOS Coordinator (中间件)         │
│  ┌────────────────────────────────────┐ │
│  │      TaskCoordinator (状态机)       │ │
│  └────────────────────────────────────┘ │
│    ↓          ↓          ↓              │
│  Navigation Vision    Arm               │
│  Adapter   Adapter  Adapter             │
└─────────────────────────────────────────┘
     ↓          ↓          ↓
  导航系统    视觉系统   机械臂
(Hybrid A*) (YOLO)   (固定姿态)
```

### 目录结构
```
mos_coordinator/
├── mos_coordinator/           # Python 包
│   ├── __init__.py
│   ├── coordinator_node.py    # 主协调器节点
│   ├── states.py              # 状态定义
│   └── adapters/              # 子系统适配器
│       ├── __init__.py
│       ├── navigation_adapter.py  # 导航适配器
│       ├── vision_adapter.py      # 视觉适配器
│       └── arm_adapter.py         # 机械臂适配器
├── config/
│   └── coordinator_params.yaml    # 配置参数
├── launch/
│   └── coordinator.launch.py      # 启动文件
├── package.xml                # ROS2 包配置
├── setup.py                   # Python 安装配置
└── README.md                  # 本文档
```

---

## 🔄 任务状态流程

```
IDLE (空闲)
  ↓ start_task()
CRUISING (巡航) ←─────────────────────┐
  ↓ 视觉检测到小车                      │
APPROACHING (接近目标)                 │
  ↓ 到达附近                           │
ALIGNING (底盘对准)                    │
  ↓ 对准完成                           │
POSITIONING (等待把手检测)             │
  ↓ 收到把手位置                       │
GRASPING (机械臂抓取)                  |
  ↓ 抓取成功                           │
TRANSPORTING (运输到卸货点)            │
  ↓ 到达卸货点                         │
RELEASING (释放小车)                  │
  ↓ 释放成功 ─────────────────────────┘
```

---

## 📡 ROS2 话题接口

### 订阅话题（输入）

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/navigation/current_pose` | `PoseStamped` | 机器人当前位姿（持续更新） |
| `/navigation/waypoint_reached` | `Bool` | 到达目标点通知 |
| `/navigation/alignment_complete` | `Bool` | 底盘对准完成通知 |
| `/vision/cart_distance` | `Float32` | 检测到的小车距离 |
| `/vision/handle_positions` | `PointStamped` | 把手位置坐标 | 暂定
| `/arm/grasp_success` | `Bool` | 抓取/释放成功通知 |

### 发布话题（输出）

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/navigation/start_patrol` | `Bool` | 开始巡航指令 |
| `/navigation/stop_patrol` | `Bool` | 停止巡航指令 |
| `/navigation/goto_pose` | `PoseStamped` | 导航到目标位姿 |
| `/navigation/request_alignment` | `Bool` | 请求底盘对准 |
| `/vision/detect_handles` | `Bool` | 请求检测把手 |
| `/arm/target_point` | `PointStamped` | 机械臂抓取目标点 | 暂定
| `/arm/release_cart` | `Bool` | 请求释放小车 |
| `/coordinator/status` | `String` | 协调器当前状态 |

---

## 🚀 快速开始

### 1. 编译
```bash
cd ~/ros2_ws
colcon build --packages-select mos_coordinator
source install/setup.bash
```

### 2. 运行
```bash
# 启动协调器节点
ros2 run mos_coordinator coordinator_node

# 或使用 launch 文件
ros2 launch mos_coordinator coordinator.launch.py
ros2 launch mos_coordinator coordinator_dummy_test.launch.py
```

### 3. 监控状态
```bash
# 查看当前状态
ros2 topic echo /coordinator/status

# 查看所有话题
ros2 topic list
```

---

## ⚙️ 配置说明

修改 `config/coordinator_params.yaml` 配置参数：

```yaml
/**:
  ros__parameters:
    # 卸货点坐标（地图坐标系）
    delivery_point:
      x: 0.0
      y: 0.0
      z: 0.0
    
    # 任务参数
    max_retries: 3
    task_timeout: 300.0
```

---

## 🔧 开发者指南

### 添加新状态
1. 在 `states.py` 中添加状态枚举
2. 在 `coordinator_node.py` 中实现状态转换逻辑
3. 更新本文档的状态流程图

### 添加新子系统适配器
1. 在 `adapters/` 创建新的适配器文件
2. 实现订阅、发布和回调函数
3. 在 `coordinator_node.py` 中初始化并调用

---

## 📝 关键决策逻辑

### 巡航点 Agent 决策链

```python
到达巡航点时:
  if 手持小车 and 到达终点:
    释放小车 → 返回巡航
  elif 手持小车:
    继续巡航（忽略新检测）
  elif 视觉检测到小车:
    停止巡航 → 接近小车
```

### 坐标计算

根据当前位姿和检测距离计算目标位置：
```python
target_x = current_x + distance * cos(yaw)
target_y = current_y + distance * sin(yaw)
```

---

## 📞 维护者

- 姓名: bolongli
- 邮箱: libolong17@gmail.com
- 文档更新日期：2026-06-30

---
