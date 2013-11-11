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


cal_accel_trim.py
=================

Removes bias from the accelerometer by recording the intial orientation of the
sensor at startup. This initial orientation is then used as the referance point
for the gravity vector for downstream filter stages.


cal_gyro_bias.py
=================

Continiously removes gyro biases with an exponential smoothing filter.


orient_accel.py
===============

Generates the orientation of the sensor from only accelerometer data


orient_gyro_simple.py
===============

Generates the orientation of the sensor from only gyro data. 
The algorithm simply integrates the rates about each axis.


orient_gyro_quaternion.py
=========================

Generates the orientation of the sensor from only gyro data. 
The algorithm generates a quaternion representation of the current rate 
measurement (dQ). dQ is then integrated to produce the resulting orientation.


orient_accgyr.py
================

Simple fusion algorithm for combining the accelerometer and gyroscope data to
form an orientation estimate. Uses a weighted quaternion SLERP for fusion of the
information sources.


orient_tf.py
============

Publish the orientation from an IMU message as a TF message.


MadgwickAHRS & MahonyAHRS
=========================

ROS wrappers for generating orientation estimates via the open source algorithms
provided at http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

The only change that was made to the upstream implementation is a dynamic update
rate. The upstream version relies on a fixed update rate.











