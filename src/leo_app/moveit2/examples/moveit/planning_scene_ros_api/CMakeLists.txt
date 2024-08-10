cmake_minimum_required(VERSION 3.8)
project(planning_scene_ros_api_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)

add_executable(planning_scene_ros_api_tutorial src/planning_scene_ros_api_tutorial.cpp)
target_include_directories(planning_scene_ros_api_tutorial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(planning_scene_ros_api_tutorial PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  planning_scene_ros_api_tutorial
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
)
install(TARGETS planning_scene_ros_api_tutorial
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
ament_package()
