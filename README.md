IMU Pipeline includes tools for processing and pre-processing IMU messages for easier use by later subscribers.

imu_transformer
===============

Transforms sensor_msgs/Imu messages into new coordinate frames using tf

imu_integrator
==============

Simple planar imu integration tool.

imu_bias_remover
================

Tool to recalibrate imu bias based on /cmd_vel or /odom