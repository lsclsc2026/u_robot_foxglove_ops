# 方式1：运行独立 Dummy 测试（推荐，无需 ROS2）
python3 /home/user/hanjiatong/mos_coordinator/mos_coordinator/dummy_coordinator_test.py

# 方式2：运行原始状态机测试
python3 -m mos_coordinator.flow_sm_test

# 方式3：运行 ROS2 节点（需安装 nav2 依赖）
#source /opt/ros/humble/setup.bash
cd work_space/ros2_ws/
source install/setup.zsh
cd -
colcon build --packages-select mos_coordinator
source install/setup.zsh
ros2 launch mos_coordinator coordinator_dummy_test.launch.py
#python3 mos_coordinator/dummy_navigator_node.py
#NAV2_AVAILABLE = False

导航使用方法：
     source  /tmp/mos_all_test_ws/unsetup_nav2.sh（注意source路径）
     source /tmp/mos_all_test_ws/setup_nav2_fastlio.sh
cd /tmp/mos_all_test_ws/
source install/setup.zsh
#注意此处可以考虑在
     ros2 launch lumos_nav nav2_fastlio_bringup.launch.py   
ros2 launch mos_arm_controller arm_controller.launch.py
export ROS_DOMAIN_ID=111
export ROS_LOCALHOST_ONLY=1
mos_slam_keyboard_teleop
ros2 run lumos_nav nav2_send_goals.py b_car
