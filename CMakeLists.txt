# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8)
project(imu_pipeline)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED roscpp sensor_msgs geometry_msgs nav_msgs tf)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs tf
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(imu_transformer src/imu_transformer.cpp)
target_link_libraries(imu_transformer ${catkin_LIBRARIES})

add_executable(imu_integrator src/imu_integrator.cpp)
target_link_libraries(imu_integrator ${catkin_LIBRARIES})

add_executable(imu_bias_remover src/imu_bias_remover.cpp)
target_link_libraries(imu_bias_remover ${catkin_LIBRARIES})

add_executable(MadgwickAHRS src/MadgwickAHRS_node.cpp)
add_library(libMadgwickAHRS src/MadgwickAHRS/MadgwickAHRS.c)
target_link_libraries(MadgwickAHRS libMadgwickAHRS ${catkin_LIBRARIES})
                                   
add_executable(MahonyAHRS src/MahonyAHRS_node.cpp)
add_library(libMahonyAHRS src/MahonyAHRS/MahonyAHRS.c)
target_link_libraries(MahonyAHRS libMahonyAHRS ${catkin_LIBRARIES})

# Install targets
install(TARGETS imu_transformer 
                imu_integrator 
                imu_bias_remover 
                MadgwickAHRS 
                MahonyAHRS
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(PROGRAMS  scripts/*.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
