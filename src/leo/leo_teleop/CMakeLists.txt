cmake_minimum_required(VERSION 3.5)
project(leo_teleop)

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
find_package(Boost REQUIRED COMPONENTS 
  system 
  thread 
  program_options
)

set(LIB_DEPENDENCIES
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "Boost"
)

add_executable(leo_teleop_node src/teleop_node.cpp)
ament_target_dependencies(leo_teleop_node ${LIB_DEPENDENCIES})

target_include_directories(leo_teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


install(TARGETS leo_teleop_node
  DESTINATION lib/${PROJECT_NAME})
install(
    PROGRAMS scripts/keyboard_control_arm.sh
    DESTINATION lib/${PROJECT_NAME})

install(
    PROGRAMS scripts/keyboard_control.sh
    DESTINATION lib/${PROJECT_NAME})
install(
    PROGRAMS scripts/cmd_location_learn_guide.sh
    DESTINATION lib/${PROJECT_NAME})
install(
    PROGRAMS scripts/location_learn_guide.sh
    DESTINATION lib/${PROJECT_NAME})

install(
    PROGRAMS scripts/cmd_location_learn_farming.sh
    DESTINATION lib/${PROJECT_NAME})
    
install(
    PROGRAMS scripts/location_learn_farming.sh
    DESTINATION lib/${PROJECT_NAME})
    
install(
    PROGRAMS scripts/object_predict.sh
    DESTINATION lib/${PROJECT_NAME})

install(
    PROGRAMS scripts/roi_hsv.sh
    DESTINATION lib/${PROJECT_NAME})    

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(
    DIRECTORY 
    launch config DESTINATION share/${PROJECT_NAME}/
)
ament_package()
