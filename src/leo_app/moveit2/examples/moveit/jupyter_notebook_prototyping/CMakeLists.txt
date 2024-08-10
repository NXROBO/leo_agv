cmake_minimum_required(VERSION 3.8)
project(jupyter_python_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)



install(DIRECTORY config launch src
	DESTINATION share/${PROJECT_NAME}
	)

ament_package()
