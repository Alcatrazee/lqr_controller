cmake_minimum_required(VERSION 3.5)
project(lqr_controller)

find_package(rclcpp REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# find_package(backward_ros REQUIRED)

nav2_package()
set(CMAKE_CXX_STANDARD 17)

include_directories(
  include
)

set(dependencies
  rclcpp
  nav2_common
  nav2_core
  nav2_util
  nav2_costmap_2d
  geometry_msgs
  nav_msgs
  nav2_msgs
  pluginlib
  tf2
  tf2_geometry_msgs
  tf2_ros
)

set(library_name lqr_controller)

add_library(${library_name} SHARED
        src/lqr_controller.cpp src/Tool.cpp src/LQR.cpp)

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY # config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core lqr_controller.xml)

ament_package()

