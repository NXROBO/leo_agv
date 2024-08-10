cmake_minimum_required(VERSION 3.8)
project(move_group_interface_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

add_executable(move_group_interface_tutorial
    src/move_group_interface_tutorial.cpp)
target_include_directories(move_group_interface_tutorial PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(move_group_interface_tutorial PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  ament_target_dependencies(
    move_group_interface_tutorial
    "moveit_ros_planning_interface"
    "rclcpp"
    "moveit_visual_tools"
  )


install(TARGETS move_group_interface_tutorial
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()