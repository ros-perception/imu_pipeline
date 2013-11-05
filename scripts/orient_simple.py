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
  
  # Calculate W
  q = []
  q.append(sqrt(1) + dot(u, v))
  
  # Calculate xyz
  a = cross(u, v)
  q.append(a[0])
  q.append(a[1])
  q.append(a[2])
  
  # Normalize q
  q  = array(q)
  q /= linalg.norm(q)
  
  return q
  
  
def quaterion_from_angles(R):
  # Finds a quaternion from rotations about each axis
  
  xaxis, yaxis, zaxis = [1, 0, 0], [0, 1, 0], [0, 0, 1]
  
  qx = quaternion_about_axis(R[0], xaxis)
  qy = quaternion_about_axis(R[1], yaxis)
  qz = quaternion_about_axis(R[2], zaxis)
  
  q = qx
  q = quaternion_multiply(q, qy)
  q = quaternion_multiply(q, qz) 

  # transformation.py stores w at the end of the list 
  q = array([ q[3], q[0], q[1], q[2] ])

  return q


def quaternion_avg(q1, q2, w1):
  
  q = w1 * q1 + (1 - w1) * q2  
  
  return q / linalg.norm(q)


def tf_quaternion_to_numpy_quarternion(q):
  return array([ q[3], q[0], q[1], q[2] ])


# Filter the messages   
def sub_gyroCB(msg_in): 
  global pub_imu
  
  global msg_prev
  global Q
    
  # Time difference
  if (msg_prev is None):
    dT = 0
  else:
    T1 = msg_prev.header.stamp
    T2 = msg_in.header.stamp
    dT = abs((T1 - T2).to_sec())
  
  # Gyro sensor vector
  x = msg_in.angular_velocity.x 
  y = msg_in.angular_velocity.y
  z = msg_in.angular_velocity.z
  sg = array([x, y, z])
  
  # Gyro Integration
  qg = quaterion_from_angles(sg * dT)
  
  
  # Gravity Vector
  g  = array([0, 0, 9.8]) 
  
  # Accel sensor vector
  x = msg_in.linear_acceleration.x 
  y = msg_in.linear_acceleration.y
  z = msg_in.linear_acceleration.z
  a = array([x, y, z])
  
  # Find a rotation between the sensor 
  # vector and the gravity vector
  qa = quaterion_between_vectors(a, g)
  
  Q = quaternion_slerp(qg, qa, 0.5)
  
  # Publish
  msg_in.orientation.w = Q[0]
  msg_in.orientation.x = Q[1]
  msg_in.orientation.y = Q[2]
  msg_in.orientation.z = Q[3]
  pub_imu.publish(msg_in)

  

if __name__ == '__main__':
  global pub_imu
  global msg_prev
  global Q
  
  msg_prev = None
  Q = zeros(4)
  
  rospy.init_node('orient_simple')
   
  pub_imu  = rospy.Publisher("/imu/orient_simple", Imu)
  
  rospy.Subscriber("/imu/trim",  Imu,  sub_ImuCB)
  rospy.spin()
  
  
  
