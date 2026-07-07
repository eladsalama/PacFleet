cmake_minimum_required(VERSION 3.8)
project(fleet_panel)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(fleet_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

add_library(fleet_panel SHARED
  src/fleet_panel.cpp
  include/fleet_panel/fleet_panel.hpp
)
target_include_directories(fleet_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fleet_panel Qt5::Widgets)
ament_target_dependencies(fleet_panel
  rclcpp
  rviz_common
  pluginlib
  fleet_interfaces
  visualization_msgs
)

install(TARGETS fleet_panel
  EXPORT export_fleet_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
ament_export_targets(export_fleet_panel)
ament_export_dependencies(rviz_common rclcpp fleet_interfaces visualization_msgs)

ament_package()
