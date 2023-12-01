cmake_minimum_required(VERSION 3.5)
project(leo_follower)

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
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Boost REQUIRED COMPONENTS 
  system 
  thread 
  program_options
)


include_directories(
  include
)

set(leo_follower_node_SRCS
  src/leo_follower.cpp

)

set(LIB_DEPENDENCIES
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "sensor_msgs"
  "PCL"
  "pcl_conversions"
)


add_executable(leo_follower_node  ${leo_follower_node_SRCS})
ament_target_dependencies(leo_follower_node ${LIB_DEPENDENCIES})



install(
  DIRECTORY 
  DESTINATION include
)

  
install(TARGETS  
  leo_follower_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)
  
ament_export_include_directories(
  include
)

ament_package()
