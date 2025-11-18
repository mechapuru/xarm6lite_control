cmake_minimum_required(VERSION 3.5)
project(xarm_moveit_servo)

# Set the RPATH to ensure the linker can find the Dynamixel SDK at runtime
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;/home/paddy/rrc/DynamixelSDK/install/dynamixel_sdk/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)

# Manually specify Dynamixel SDK paths
include_directories(/home/paddy/rrc/DynamixelSDK/c++/include)
link_directories(/home/paddy/rrc/DynamixelSDK/install/dynamixel_sdk/lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  control_msgs
  moveit_msgs
  geometry_msgs
  moveit_ros_planning
)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

add_library(xarm_joystick_input SHARED 
  src/xarm_joystick_input.cpp
  src/gripper_controller.cpp
)
ament_target_dependencies(xarm_joystick_input ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Link the dynamixel_sdk library directly by name
target_link_libraries(xarm_joystick_input dynamixel_sdk)
rclcpp_components_register_nodes(xarm_joystick_input "xarm_moveit_servo::JoyToServoPub")

add_executable(xarm_keyboard_input
  src/xarm_keyboard_input.cpp
)
ament_target_dependencies(xarm_keyboard_input ${THIS_PACKAGE_INCLUDE_DEPENDS})


install(TARGETS
  xarm_joystick_input
  xarm_keyboard_input
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
