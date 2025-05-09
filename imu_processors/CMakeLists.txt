cmake_minimum_required(VERSION 3.5)
project(imu_processors)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(imu_integrator SHARED
  src/imu_integrator.cpp
)
target_link_libraries(imu_integrator PUBLIC
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  sensor_msgs::sensor_msgs_library
)
rclcpp_components_register_node(imu_integrator
  PLUGIN "imu_processors::ImuIntegrator"
  EXECUTABLE imu_integrator_node
)

add_library(imu_bias_remover SHARED
  src/imu_bias_remover.cpp
)
target_link_libraries(imu_bias_remover PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  sensor_msgs::sensor_msgs_library
)
rclcpp_components_register_node(imu_bias_remover
  PLUGIN "imu_processors::ImuBiasRemover"
  EXECUTABLE imu_bias_remover_node
)

if (BUILD_TESTING)
  find_package(ament_cmake_cpplint)
  ament_cpplint()
endif()

install(
  TARGETS
    imu_bias_remover
    imu_integrator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(
  imu_bias_remover
  imu_integrator
)
ament_package()
