cmake_minimum_required(VERSION 3.8)
project(demo_vehicle_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake_auto REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

ament_auto_find_build_dependencies()

add_executable(demo_vehicle_interface_node src/demo_vehicle_interface_node.cpp)

target_include_directories(demo_vehicle_interface_node PUBLIC include)

ament_target_dependencies(demo_vehicle_interface_node
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  rclcpp
)

install(TARGETS
  demo_vehicle_interface_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)