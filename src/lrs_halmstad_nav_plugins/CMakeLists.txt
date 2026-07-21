cmake_minimum_required(VERSION 3.8)
project(lrs_halmstad_nav_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lrs_halmstad_interfaces REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)

add_library(aerial_support_layer SHARED
  src/aerial_support_layer.cpp
)
target_include_directories(aerial_support_layer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(aerial_support_layer
  builtin_interfaces
  geometry_msgs
  lrs_halmstad_interfaces
  nav2_costmap_2d
  pluginlib
  rcl_interfaces
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
  vision_msgs
)

pluginlib_export_plugin_description_file(
  nav2_costmap_2d
  aerial_support_layer_plugin.xml
)

install(
  TARGETS aerial_support_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  FILES aerial_support_layer_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nav2_util REQUIRED)

  ament_add_gtest(test_aerial_support_layer
    test/test_aerial_support_layer.cpp
    SKIP_LINKING_MAIN_LIBRARIES
  )
  target_link_libraries(test_aerial_support_layer aerial_support_layer)
  ament_target_dependencies(test_aerial_support_layer
    builtin_interfaces
    geometry_msgs
    lrs_halmstad_interfaces
    nav2_costmap_2d
    nav2_util
    pluginlib
    rclcpp
    rclcpp_lifecycle
    tf2
    tf2_ros
    vision_msgs
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(aerial_support_layer)
ament_export_dependencies(
  builtin_interfaces
  geometry_msgs
  lrs_halmstad_interfaces
  nav2_costmap_2d
  pluginlib
  rcl_interfaces
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
  vision_msgs
)
ament_package()
