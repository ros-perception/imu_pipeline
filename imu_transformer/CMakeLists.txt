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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

include_directories(
  include
)

add_executable(imu_transformer_node src/imu_transformer_node.cpp src/imu_transformer.cpp)
ament_target_dependencies(imu_transformer_node rclcpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_sensor_msgs)

#if(BUILD_TESTING)
  #find_package(tf2_geometry_msgs REQUIRED)
  #find_package(ament_cmake_gtest REQUIRED)

  #ament_add_gtest(test_imu_transforms test/test_imu_transforms.cpp)
  #ament_target_dependencies(test_imu_transforms sensor_msgs geometry_msgs tf2 tf2_ros tf2_sensor_msgs)
#endif()

install(TARGETS
  imu_transformer_node
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
