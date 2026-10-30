cmake_minimum_required(VERSION 3.16)
project(rm_aim_sim_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AimCommand.msg"
  "msg/ArmorCorners.msg"
  "msg/ArmorCornersArray.msg"
  "msg/GimbalState.msg"
  "msg/Telemetry.msg"
  DEPENDENCIES std_msgs geometry_msgs)
rosidl_get_typesupport_target(command_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_export_dependencies(rosidl_default_runtime)

add_executable(bridge src/bridge_node.cpp src/mailbox.cpp)
target_include_directories(bridge PRIVATE include)
target_link_libraries(bridge nlohmann_json::nlohmann_json "${command_typesupport}")
ament_target_dependencies(bridge rclcpp rcl_interfaces builtin_interfaces geometry_msgs
  std_msgs rosgraph_msgs)

install(TARGETS bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control_gate test/test_control_gate.cpp)
  target_include_directories(test_control_gate PRIVATE include)
  target_link_libraries(test_control_gate nlohmann_json::nlohmann_json)
  ament_add_gtest(test_ros_bridge test/test_ros_bridge.cpp
    APPEND_LIBRARY_DIRS "${CMAKE_CURRENT_BINARY_DIR}")
  ament_target_dependencies(test_ros_bridge rclcpp geometry_msgs std_msgs
    rosgraph_msgs)
  target_link_libraries(test_ros_bridge nlohmann_json::nlohmann_json "${command_typesupport}")
  target_compile_definitions(test_ros_bridge PRIVATE
    BRIDGE_EXECUTABLE="$<TARGET_FILE:bridge>" SIM_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../../..")
  add_dependencies(test_ros_bridge bridge)
endif()

ament_package()
