cmake_minimum_required(VERSION 3.5)
project(imu_transformer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

include_directories(
  include
)

add_library(imu_transformer SHARED
  src/imu_transformer.cpp
)
ament_target_dependencies(imu_transformer
  message_filters
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
)
rclcpp_components_register_node(imu_transformer
  PLUGIN "imu_transformer::ImuTransformer"
  EXECUTABLE imu_transformer_node
)

if(BUILD_TESTING)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_imu_transforms test/test_imu_transforms.cpp)
  ament_target_dependencies(test_imu_transforms sensor_msgs geometry_msgs tf2 tf2_ros tf2_sensor_msgs tf2_geometry_msgs)
endif()

install(
  TARGETS imu_transformer
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
