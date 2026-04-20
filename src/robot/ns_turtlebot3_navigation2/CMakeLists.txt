################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(ns_turtlebot3_navigation2)


################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)




################################################################################
# Install
################################################################################
install(DIRECTORY launch models rviz urdf params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
