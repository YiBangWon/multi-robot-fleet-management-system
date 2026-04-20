cmake_minimum_required(VERSION 3.8)
project(fms_custom_tb)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

set(testbed_asset_directories
  launch
  models
  worlds
  maps
  rviz
)

foreach(asset_dir IN LISTS testbed_asset_directories)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${asset_dir}")
    install(
      DIRECTORY ${asset_dir}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/fms_custom_tb.dsv.in")

ament_export_dependencies(ament_cmake)
ament_export_dependencies(gazebo_ros)

ament_package()
