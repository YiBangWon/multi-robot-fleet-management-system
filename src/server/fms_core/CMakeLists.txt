cmake_minimum_required(VERSION 3.5)
project(fms_core)

set(CMAKE_BUILD_TYPE Debug)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_compile_options(-std=c++14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
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

find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

file(GLOB SOURCES "src/fleet_manager/*.cpp")
file(GLOB ECBS_SOURCES "src/ECBS/*.cpp")
find_package(Boost 1.58 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories( ${Boost_INCLUDE_DIRS} )

add_executable(fms_core_node ${SOURCES} ${ECBS_SOURCES})
ament_target_dependencies(fms_core_node
  ${Boost_LIBRARIES}
  Eigen3
  rclcpp std_msgs visualization_msgs nav_msgs fms_msgs tf2 tf2_ros tf2_geometry_msgs
  yaml-cpp
)
target_link_libraries(fms_core_node
  yaml-cpp
)

install(TARGETS fms_core_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
