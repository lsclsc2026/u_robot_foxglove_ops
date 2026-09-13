# 方式1：运行独立 Dummy 测试（推荐，无需 ROS2）
python3 /home/user/hanjiatong/mos_coordinator/mos_coordinator/dummy_coordinator_test.py

# 方式2：运行原始状态机测试
python3 -m mos_coordinator.flow_sm_test

# 方式3：运行 ROS2 节点（需安装 nav2 依赖）
source /opt/ros/humble/setup.bash
colcon build --packages-select mos_coordinator
ros2 launch mos_coordinator coordinator_dummy_test.launch.py