cmake_minimum_required(VERSION 3.8)
project(rog_map_costmap_layer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(rog_map_costmap_layer SHARED src/rog_map_layer.cpp)
target_include_directories(rog_map_costmap_layer PUBLIC include)
ament_target_dependencies(rog_map_costmap_layer
  nav2_costmap_2d nav_msgs pluginlib rclcpp tf2 tf2_geometry_msgs tf2_ros)

pluginlib_export_plugin_description_file(nav2_costmap_2d rog_map_costmap_layer.xml)

install(TARGETS rog_map_costmap_layer LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES rog_map_costmap_layer.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(rog_map_costmap_layer)
ament_package()
