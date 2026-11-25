cmake_minimum_required(VERSION 3.10)
project(small_point_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-O3 -Wall -Wextra -Wno-unused-parameter -ffast-math -fno-math-errno)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message(STATUS "Processor number:  ${N}")

  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message(STATUS "core for MP:  3")
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message(STATUS "core for MP:  ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(OpenMP QUIET)
configure_file(include/param_deliver.h.in include/param_deliver.h @ONLY)

file(GLOB SMALL_POINT_LIO_SOURCES
  src/small_point_lio/*.cpp
  src/io/*.cpp
  src/util/*.cpp
  src/small_point_lio_node.cpp
  3rdparty/liblzf/*.c)

add_executable(small_point_lio_node ${SMALL_POINT_LIO_SOURCES})
target_include_directories(small_point_lio_node PRIVATE
  include src 3rdparty ${CMAKE_CURRENT_BINARY_DIR}/include)
if(OPENMP_FOUND)
  target_link_libraries(small_point_lio_node OpenMP::OpenMP_CXX)
endif()
target_link_libraries(small_point_lio_node Eigen3::Eigen)
ament_target_dependencies(small_point_lio_node rclcpp rclcpp_components sensor_msgs nav_msgs livox_ros_driver2)

# Install the executable
install(TARGETS small_point_lio_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Export dependencies
ament_export_dependencies(
  rclcpp
  nav_msgs
  sensor_msgs
  rclcpp_components
  livox_ros_driver2
  Eigen3
)

ament_package()
