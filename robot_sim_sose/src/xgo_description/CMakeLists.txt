cmake_minimum_required(VERSION 3.5)
project(xgo_description)

# Default to C++14 if not specified
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find the ROS 2 build tool
find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)

add_library(xgo_glide_trajectory_system SHARED
  src/GlideTrajectorySystem.cc
)
target_link_libraries(xgo_glide_trajectory_system
  PRIVATE gz-sim8::core
)

install(TARGETS xgo_glide_trajectory_system
  LIBRARY DESTINATION lib
)


# Install runtime assets in the package share directory.
install(DIRECTORY
    config
    launch
    rviz
    scripts
    worlds
    urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
