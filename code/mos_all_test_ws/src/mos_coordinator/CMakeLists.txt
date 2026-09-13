cmake_minimum_required(VERSION 3.8)
project(mos_coordinator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveChassis.srv"
  "srv/RunTask.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Python package installation is handled by setup.py
# (ament_cmake_python will detect and use setup.py automatically)

# Manually install Python module files to dist-packages
install(DIRECTORY mos_coordinator/
  DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

# Install Python executables explicitly
install(PROGRAMS
  mos_coordinator/coordinator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME coordinator_node
)

install(PROGRAMS
  mos_coordinator/simulator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME simulator_node
)

install(PROGRAMS
  mos_coordinator/dummy_3d_object_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dummy_3d_object_node
)

install(PROGRAMS
  mos_coordinator/dummy_cart_fullness_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dummy_cart_fullness_node
)

install(PROGRAMS
  mos_coordinator/dummy_grasp_selector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dummy_grasp_selector_node
)

install(PROGRAMS
  mos_coordinator/dummy_chassis_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dummy_chassis_node
)

install(PROGRAMS
  mos_coordinator/dummy_arm_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dummy_arm_node
)

install(PROGRAMS
  mos_coordinator/dummy_navigator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dummy_navigator_node
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


