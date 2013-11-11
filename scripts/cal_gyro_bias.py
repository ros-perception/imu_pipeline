#!/usr/bin/env python
# 
# Gyroscope bias filter - Exponential Moving Average
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


# http://en.wikipedia.org/wiki/Exponential_smoothing
def update(oldval, newval):
  alpha = 0.995
  oldval = alpha * oldval + (1.0 - alpha) * newval
  return oldval

  
def sub_imuCB(msg_in): 
  global pub_imu
  global prev_x
  global prev_y
  global prev_z
  
  msg_in.angular_velocity.x -= update(prev_x, msg_in.angular_velocity.x)
  msg_in.angular_velocity.y -= update(prev_y, msg_in.angular_velocity.y)
  msg_in.angular_velocity.z -= update(prev_z, msg_in.angular_velocity.z)
  
  pub_imu.publish(msg_in)


if __name__ == '__main__':
  global pub_imu
  global prev_x
  global prev_y
  global prev_z
  
  rospy.init_node('imu_bias_exp')
   
  pub_imu  = rospy.Publisher("/imu/bias", Imu)
  
  prev_x = 0.0
  prev_y = 0.0
  prev_z = 0.0
  
  rospy.Subscriber("/android/imu", Imu,  sub_imuCB)
  rospy.spin()
  
  
  
