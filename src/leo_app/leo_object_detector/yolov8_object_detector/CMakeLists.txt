cmake_minimum_required(VERSION 3.5)
project(yolov8_object_detector)

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


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    rviz
    scripts
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
  scripts/yolov8n.pt
  scripts/yolov8_ros2_yolov8n_pt.py
  launch/start_yolov8.launch.py
  launch/leo_yolov8_object_detector_teleop.launch.py

  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
