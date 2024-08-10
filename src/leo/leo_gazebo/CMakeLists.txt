cmake_minimum_required(VERSION 3.5)
project(leo_gazebo)

find_package(ament_cmake REQUIRED)

# Install files
install(DIRECTORY 
        launch 
        meshes 
        rviz 
        urdf
        world
        
        DESTINATION share/${PROJECT_NAME})

ament_package()
