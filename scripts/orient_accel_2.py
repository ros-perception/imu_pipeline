#!/usr/bin/env python
# 
# Orientation Filter - Accelerometer data only
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
from tf.transformations import *
from sensor_msgs.msg  import Imu

import math
from numpy import *

def quaterion_between_vectors(u, v):
  # Finds the shortest rotation between two vectors
  # expressed as a Quarternion. Note that there are
  # many solutions at u = -v, so this causes some 
  # weird behavior due to the fact we have no rotation
  # constraint about the vectors.
  
  # Normalize input vectors
  u_norm = linalg.norm(u)
  v_norm = linalg.norm(v)
  u /= u_norm
  v /= v_norm  
  
  # Calculate xyz
  a = cross(u, v)
  
  q = []
  q.append(a[0])
  q.append(a[1])
  q.append(a[2])
  
  # Calculate W
  q.append(1 + dot(u, v))
  
  # Normalize q
  q  = array(q)
  q /= linalg.norm(q)
  
  return q


def rotate_point(Q, p):
  return quaternion_multiply(quaternion_multiply(Q, p), quaternion_conjugate(Q))
  
  
# Filter the messages   
def sub_imuCB(msg_in): 
  global pub_imu
  global Q
  
  # Current rotation from world frame (_wf) to sensor frame (_sf)
  Q_wf2sf = quaternion_inverse(Q)
  Q_sf2wf = Q
  
  # Gravity vector in world frame 
  g_wf = array([0, 0, 9.8, 0]) 
  
  # Sensor vector in sensor frame 
  x = msg_in.linear_acceleration.x 
  y = msg_in.linear_acceleration.y
  z = msg_in.linear_acceleration.z
  s_sf = array([x, y, z, 0])
  
  
  # Find the gravity vector in the sensor frame
  g_sf = rotate_point(Q_wf2sf, g_wf)
  
  # Find the sensor data in the world frame
  s_wf = rotate_point(Q_sf2wf, s_sf)
  
  #print s_sf[:3], g_wf[:3]
    
  q = quaterion_between_vectors(s_wf[:3], g_wf[:3])
  Q = quaternion_multiply(q, Q)
  print q
  
  
  # Publish
  msg_in.orientation.x = Q[0]
  msg_in.orientation.y = Q[1]
  msg_in.orientation.z = Q[2]
  msg_in.orientation.w = Q[3]
  pub_imu.publish(msg_in)


if __name__ == '__main__':
  global pub_imu
  global Q
  
  Q = array([0, 0, 0, 1])
  Q = quaternion_about_axis(pi/4, [1, 1, 0])
    
  set_printoptions(precision=3)
  set_printoptions(suppress=True)
  
  rospy.init_node('orient_accel')
   
  pub_imu  = rospy.Publisher("/imu/orient", Imu)
  
  rospy.Subscriber("/imu/trim", Imu,  sub_imuCB)
  rospy.spin()
  
  
  
