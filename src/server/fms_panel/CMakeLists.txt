cmake_minimum_required(VERSION 3.5)

project(fms_panel LANGUAGES CXX)

# 기본 C++ 표준 설정
set(CMAKE_CXX_STANDARD 14)

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(fms_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_assimp_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

# 헤더 파일과 소스 파일 설정
set(${PROJECT_NAME}_headers_to_moc include/fms_panel/fms_panel.h)
qt5_wrap_cpp(${PROJECT_NAME}_moc_files ${${PROJECT_NAME}_headers_to_moc})

set(${PROJECT_NAME}_source_files src/fms_panel.cpp)

add_library(${PROJECT_NAME} SHARED
    ${${PROJECT_NAME}_moc_files}
    ${${PROJECT_NAME}_source_files}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    rviz_ogre_vendor::OgreMain
    rviz_ogre_vendor::OgreOverlay
    Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME}
    PUBLIC
    fms_msgs
    pluginlib
    rclcpp
    rviz_assimp_vendor
    rviz_rendering
    rviz_common
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(
    DIRECTORY include/
    DESTINATION include
)

ament_package()
