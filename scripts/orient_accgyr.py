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
import tf
from sensor_msgs.msg  import Imu

import math
from numpy import *
from tf.transformations import *

  
  
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
  
  

def quaterion_between_vectors(u, v):
  # Finds the shortest rotation between two vectors
  # expressed as a Quarternion. 
  
  # Normalize input vectors
  u_norm = linalg.norm(u)
  v_norm = linalg.norm(v)
  u /= u_norm
  v /= v_norm  
  
  # Calculate xyz 
  # (Axis of rotation is normal to both vectors)
  a = cross(u, v)
  q = [0, 0, 0]
  q[0] = a[0]
  q[1] = a[1]
  q[2] = a[2]  
  
  # Calculate W
  # (The half-angle between both vectors)
  q.append(1 + dot(u, v))
  
  # Normalize q
  q  = array(q)
  q /= linalg.norm(q)
  
  return q


def rotate_point(Q, p):
  # P = Q * p * Q ** -1
  return quaternion_multiply(quaternion_multiply(Q, p), quaternion_conjugate(Q))
  

# Filter the messages   
def sub_imuCB(msg_in): 
  global pub_imu
  
  global msg_prev
  global Q
    
  ##
  ## Gyroscope Update
  ##
  
  # Time difference
  if (msg_prev is None):
    dT = 0
  else:
    T1 = msg_prev.header.stamp
    T2 = msg_in.header.stamp
    dT = abs((T1 - T2).to_sec())
  
  # Gyro sensor vector
  gx = msg_in.angular_velocity.x 
  gy = msg_in.angular_velocity.y
  gz = msg_in.angular_velocity.z
  g = array([gx, gy, gz])
  
  # Translate to Quarternion  
  q = quaterion_from_angles(g * dT)
  
  # Integration of gyro data
  Qg = quaternion_multiply(Q, q)
  

  ##
  ## Accelerometer Update
  ##

  # Current rotation from world frame (_wf) to sensor frame (_sf)
  Q_wf2sf = quaternion_inverse(Q)
  Q_sf2wf = Q
  
  # Gravity vector in world frame 
  g_wf = array([0, 0, 9.8, 0]) 
  
  # Sensor vector in sensor frame 
  ax = msg_in.linear_acceleration.x 
  ay = msg_in.linear_acceleration.y
  az = msg_in.linear_acceleration.z
  s_sf = array([ax, ay, az, 0])
  
  # Find the sensor data in the world frame
  s_wf = rotate_point(Q_sf2wf, s_sf)
  
  q = quaterion_between_vectors(s_wf[:3], g_wf[:3])
  Qa = quaternion_multiply(q, Q)
  
 
  ##
  ## Publish
  ##
  Q = quaternion_slerp(Qg, Qa, 0.01)
  msg_prev = msg_in
  msg_in.orientation.w = Q[3]
  msg_in.orientation.x = Q[0]
  msg_in.orientation.y = Q[1]
  msg_in.orientation.z = Q[2]
  pub_imu.publish(msg_in)

  

if __name__ == '__main__':
  global pub_imu
  global msg_prev
  global Q
  
  msg_prev = None
  Q = array([0,0,0,1])
  
  set_printoptions(precision=3)
  set_printoptions(suppress=True)
  
  rospy.init_node('orient_simple')
   
  pub_imu  = rospy.Publisher("/imu/orient", Imu)
  
  rospy.Subscriber("/imu/trim",  Imu,  sub_imuCB)
  rospy.spin()
  
  
  
