cmake_minimum_required(VERSION 3.5)
project(robot_manager)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(fms_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(Eigen3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

if(NOT Eigen3_INCLUDE_DIRS)
  set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)
include_directories( ${Boost_INCLUDE_DIRS} )


add_executable(fms_ctrl_node src/fms_ctrl_node.cpp src/robot_manager.cpp src/controller.cpp src/dwa.cpp)
ament_target_dependencies(fms_ctrl_node
  Eigen3 rclcpp std_msgs visualization_msgs nav_msgs nav2_msgs fms_msgs tf2 tf2_ros tf2_geometry_msgs geometry_msgs
)

add_executable(fms_single_ctrl_node src/fms_single_ctrl_node.cpp src/robot_manager.cpp src/controller.cpp src/dwa.cpp)
ament_target_dependencies(fms_single_ctrl_node
  Eigen3 rclcpp std_msgs visualization_msgs nav_msgs nav2_msgs fms_msgs tf2 tf2_ros tf2_geometry_msgs geometry_msgs
)

install(TARGETS fms_ctrl_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS fms_single_ctrl_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
