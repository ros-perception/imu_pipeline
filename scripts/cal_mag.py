#!/usr/bin/env python
# 
# Calibrate Magnetometer - Hard and Soft Iron Effects
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
from sensor_msgs.msg  import MagneticField
from sensor_msgs.msg  import PointCloud2
from sensor_msgs.msg  import PointField

import math
from numpy import *

from IPython import embed

def rotate_point(Q, p):
  return quaternion_multiply(quaternion_multiply(Q, p), quaternion_conjugate(Q))


# Get current orientation of IMU  
def sub_imuCB(msg_in): 
  global Q

  Q[0] = msg_in.orientation.x
  Q[1] = msg_in.orientation.y
  Q[2] = msg_in.orientation.z
  Q[3] = msg_in.orientation.w


# Filter the messages
def sub_magCB(msg_in):
  global pub_imu, pub_pc, msg_pc, pc
  global Q
  
  # Read sensor data
  mx = msg_in.magnetic_field.x
  my = msg_in.magnetic_field.y
  mz = msg_in.magnetic_field.z
  m  = [mx, my, mz]
  
  # Allocate storage for point
  if (msg_pc.width < 100):
    msg_pc.width += 1
    pc.resize(msg_pc.width)
    print msg_pc.width
  else:
    embed()
  
  # Store point in cloud
  pc[msg_pc.width - 1] = (m[0], m[1], m[2])
  
  # Publish
  msg_pc.header = msg_in.header
  msg_pc.data   = atleast_2d(pc).tostring()
  pub_pc.publish(msg_pc) 


if __name__ == '__main__':
  global pub_imu, pub_pc, msg_pc
  global Q, pc
  
  # Current device orientation
  Q = array([0, 0, 0, 1])
    
  # Init Pointcloud
  msg_pc = PointCloud2()
  msg_pc.height = 1
  msg_pc.width  = 0
  
  msg_pc.fields.append(PointField())
  msg_pc.fields[0].name     = 'x'
  msg_pc.fields[0].offset   = 0
  msg_pc.fields[0].datatype = PointField.FLOAT32
  msg_pc.fields[0].count    = 1
  
  msg_pc.fields.append(PointField())
  msg_pc.fields[1].name     = 'y'
  msg_pc.fields[1].offset   = 4
  msg_pc.fields[1].datatype = PointField.FLOAT32
  msg_pc.fields[1].count    = 1
  
  msg_pc.fields.append(PointField())
  msg_pc.fields[2].name     = 'z'
  msg_pc.fields[2].offset   = 8
  msg_pc.fields[2].datatype = PointField.FLOAT32
  msg_pc.fields[2].count    = 1

  msg_pc.is_bigendian = False
  msg_pc.point_step   = 3 * 4
  msg_pc.is_dense     = True

  pc = zeros((0), dtype=[('x', float32), ('y', float32), ('z', float32)])

    
  # numpy printing options
  set_printoptions(precision=3)
  set_printoptions(suppress=True)
  
  # Start the ROS node
  rospy.init_node('cal_mag')
   
  pub_imu  = rospy.Publisher("/imu/magnetic_field/calibrated", Imu)
  pub_pc   = rospy.Publisher("/imu/magnetic_field/pointcloud", PointCloud2)
  
  rospy.Subscriber("/android/magnetic_field", MagneticField,  sub_magCB)
  rospy.Subscriber("/imu/orient"            ,           Imu,  sub_imuCB)
  rospy.spin()
  
  
  
