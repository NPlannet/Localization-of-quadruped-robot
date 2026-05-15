cmake_minimum_required(VERSION 3.5)
project(xgo_description)

# Default to C++14 if not specified
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find the ROS 2 build tool
find_package(ament_cmake REQUIRED)

# Install directories to the 'share' folder so ROS 2 can find your meshes and URDF
install(DIRECTORY 
    config 
    launch 
    meshes 
    worlds
    urdf
  DESTINATION share/${PROJECT_NAME}
)

# This replaces catkin_package() and MUST be the last line
ament_package()
