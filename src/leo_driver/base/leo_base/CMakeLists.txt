cmake_minimum_required(VERSION 3.5)
project(leo_base)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS 
  system 
  thread 
  program_options
)
set(msg_files
  "msg/GyroMessage.msg"
  "msg/LeoBaseOdom.msg"
  "msg/LeoBaseDock.msg"
  "msg/LeoBaseSensor.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)


include_directories(
  include
)

set(leo_base_node_SRCS
  src/leo_base_serial.cpp
  src/leo_base_interface.cpp
  src/leo_base_driver.cpp
  src/leo_base_node.cpp

)

set(LIB_DEPENDENCIES
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "sensor_msgs"

)


add_executable(leo_base_node  ${leo_base_node_SRCS})
ament_target_dependencies(leo_base_node ${LIB_DEPENDENCIES})

rosidl_target_interfaces(leo_base_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(
  DIRECTORY include/
  DESTINATION include
)

  
install(TARGETS  
leo_base_node  
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)
  
ament_export_include_directories(
  include
)

ament_package()
