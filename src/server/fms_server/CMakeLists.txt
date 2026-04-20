cmake_minimum_required(VERSION 3.5)
project(fms_server)

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

install(DIRECTORY launch envs exe cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

