IMU Pipeline metapackage includes tools for processing and pre-processing IMU messages for easier use by later subscribers.

Branches
========

 * ``ros2`` - main branch, for ROS 2 Kilted and later
 * ``jazzy`` - supports ROS 2 Humble to Jazzy

imu_transformer
===============

Transforms sensor_msgs/Imu and sensor_msgs/MagneticField messages into new coordinate frames using tf2

imu_processors
==============

Includes misc preprocessors for IMU data:
* imu_integrator: Simple planar imu integration tool.
* imu_bias_remover: Tool to recalibrate imu bias based on /cmd_vel or /odom
