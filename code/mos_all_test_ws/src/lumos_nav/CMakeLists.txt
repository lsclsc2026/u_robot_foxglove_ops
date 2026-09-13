cmake_minimum_required(VERSION 3.5)
project(lumos_nav)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install params files
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

# Install rviz files
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install scripts
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME}/lib
  USE_SOURCE_PERMISSIONS)

# Install TF bridge executable to lib/ for ros2 launch
install(PROGRAMS scripts/localization_tf_bridge.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/cloud_stamp_bridge.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/goal_arrival_monitor.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/wait_for_map_to_odom.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/cmd_vel_to_base.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/scan_frame_bridge.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/nav2_get_pose.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/nav2_send_goals.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/nav_arm_grasp_demo.py
  DESTINATION lib/${PROJECT_NAME})

# Create symlink for prebuilt purepursuit_follower ARM64 binary
install(CODE
  "file(MAKE_DIRECTORY \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/lumos_nav)")
install(CODE
  "execute_process(COMMAND
    ln -sf /opt/slam/apps/install/lib/lumos_nav/purepursuit_follower
    \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/lumos_nav/purepursuit_follower)")
install(CODE "message(STATUS \"Created symlink for purepursuit_follower\")")

ament_package()
