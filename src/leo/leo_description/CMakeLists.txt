cmake_minimum_required(VERSION 3.5)
project(leo_description)

find_package(ament_cmake REQUIRED)

# Install files
install(DIRECTORY 
        launch 
        meshes 
        rviz 
        urdf
        
        DESTINATION share/${PROJECT_NAME})

ament_package()
