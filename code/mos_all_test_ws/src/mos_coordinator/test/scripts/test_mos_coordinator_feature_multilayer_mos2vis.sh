#!/usr/bin
#open a window:
#导航使用方法：
source  /tmp/mos_all_test_ws/unsetup_nav2.sh（注意source路径）
source /tmp/mos_all_test_ws/setup_nav2_fastlio.sh
cd /tmp/mos_all_test_ws/
source install/setup.zsh
ros2 launch lumos_nav nav2_fastlio_bringup.launch.py   

#open another window:
#ros2 launch mos_arm_controller arm_controller.launch.py # attention if your mos_coordinator launch arm_controller, then dont open another one.

#open another window:
source ~/work_space/ros2_ws/install/setup.zsh
mos_slam_keyboard_teleop

#open another window:
source  /tmp/mos_all_test_ws/unsetup_nav2.sh（注意source路径）
source /tmp/mos_all_test_ws/setup_nav2_fastlio.sh
cd /tmp/mos_all_test_ws/
source install/setup.zsh
ros2 run lumos_nav nav2_send_goals.py b_car

#open another window:
cd /tmp/mos_all_test_ws/
export ROS_DOMAIN_ID=111
export ROS_LOCALHOST_ONLY=1
cp src/mos_coordinator/config/coordinator_params_mos2_2607211500.yaml src/mos_coordinator/config/coordinator_params.yaml
colcon build --packages-select mos_coordinator
ros2 launch mos_coordinator coordinator_mos_test.launch.py | tee mos_test.2607220030.txt
#ros2 launch mos_coordinator coordinator_mos_test.launch.py | tee mos_test.2607220110.txt
