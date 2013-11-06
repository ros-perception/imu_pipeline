#!/usr/bin/env python
# 
# Orientation Filter - Gyroscope data only (Angle based)
#
# Copyright (c) 2013 Dereck Wonnacott <dereck@gmail.com>
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# 

import rospy
from sensor_msgs.msg  import Imu
from tf.transformations import *
import math
from numpy import *


def quaterion_from_angles(R):
  # Finds a quaternion from rotations about each axis
  
  xaxis, yaxis, zaxis = [1, 0, 0], [0, 1, 0], [0, 0, 1]
  
  qx = quaternion_about_axis(R[0], xaxis)
  qy = quaternion_about_axis(R[1], yaxis)
  qz = quaternion_about_axis(R[2], zaxis)
  
  q = qx
  q = quaternion_multiply(q, qy)
  q = quaternion_multiply(q, qz)

  return q

  
# Filter the messages   
def sub_imuCB(msg_in): 
  global pub_imu
  
  global msg_prev
  global R
    
  # Sensor vector
  x = msg_in.angular_velocity.x 
  y = msg_in.angular_velocity.y
  z = msg_in.angular_velocity.z
  s = array([x, y, z])
  
  # Time difference
  if (msg_prev is None):
    dT = 0
  else:
    T1 = msg_prev.header.stamp
    T2 = msg_in.header.stamp
    dT = abs((T1 - T2).to_sec())
  
  # Integration
  R = R + s * dT
  
  # Translate to Quarternion  
  q = quaterion_from_angles(R)
  
  print q
  
  # Publish
  msg_prev = msg_in
  msg_in.orientation.w = q[3]
  msg_in.orientation.x = q[0]
  msg_in.orientation.y = q[1]
  msg_in.orientation.z = q[2]
  pub_imu.publish(msg_in)


if __name__ == '__main__':
  global pub_imu
  
  global msg_prev
  global R
  
  msg_prev = None
  R = array([0,0,0])
  
  set_printoptions(precision=4)
  
  rospy.init_node('orient_gyro')
   
  pub_imu  = rospy.Publisher("/imu/orient_gyro", Imu)
  
  rospy.Subscriber("/imu/bias", Imu,  sub_imuCB)
  rospy.spin()
  
  
  
