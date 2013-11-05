#!/usr/bin/env python
# 
# Accelerometer Trim Filter - Exponential Moving Averate at startup
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

import math

# http://en.wikipedia.org/wiki/Exponential_smoothing
def update(oldval, newval):
  alpha = 0.5
  oldval = alpha * oldval + (1.0 - alpha) * newval
  return oldval
  
  
# Filter the messages   
def sub_imuCB(msg_in): 
  global pub_imu
  global trim_x
  global trim_y
  global trim_z
  global prev_x
  global prev_y
  global prev_z
  
  auto_calibration_threshold = 0.001    # Gravity vector
  g  = 9.8   
  gx = math.pi / 2
  gy = math.pi / 2
  gz = 0
  
  if(trim_x is None or trim_y is None or trim_z is None):  
    # Autocalibration finds the average of the three sensor axes
    # and waits for it to settle before starting the filter. 
  
    # Sensor vector
    x = msg_in.linear_acceleration.x 
    y = msg_in.linear_acceleration.y
    z = msg_in.linear_acceleration.z
  
    sr = math.sqrt(z**2 + y**2 + x**2)
    sx = math.acos(x/sr)
    sy = math.acos(y/sr)
    sz = math.acos(z/sr)
    
    # Update Filter
    prev_prev_x = prev_x
    prev_prev_y = prev_y
    prev_prev_z = prev_z
    
    prev_x = update(prev_x, sx - gx)
    prev_y = update(prev_y, sy - gy)
    prev_z = update(prev_z, sz - gz)
    
    # Check if the filter has stabilized
    if (abs(prev_prev_x - prev_x) < auto_calibration_threshold):
      trim_x = prev_x
    
    if (abs(prev_prev_y - prev_y) < auto_calibration_threshold):
      trim_y = prev_y
      
    if (abs(prev_prev_z - prev_z) < auto_calibration_threshold):
      trim_z = prev_z
    
    if(trim_x is not None and trim_y is not None and trim_z is not None):
      rospy.loginfo("Trim calibration complete.")
  
  else:
    # Rotate the sensor vector by the trim vector
    
    # Sensor vector
    x = msg_in.linear_acceleration.x 
    y = msg_in.linear_acceleration.y
    z = msg_in.linear_acceleration.z
  
    sr = math.sqrt(z**2 + y**2 + x**2)
    sx = math.acos(x/sr)
    sy = math.acos(y/sr)
    sz = math.acos(z/sr)
    
    # Output Vector
    msg_in.linear_acceleration.x = sr * math.cos(sx - trim_x) 
    msg_in.linear_acceleration.y = sr * math.cos(sy - trim_y)
    msg_in.linear_acceleration.z = sr * math.cos(sz - trim_z)
  
  pub_imu.publish(msg_in)


if __name__ == '__main__':
  global pub_imu
  global trim_x
  global trim_y
  global trim_z
  global prev_x
  global prev_y
  global prev_z
  
  rospy.init_node('imu_trim')
   
  pub_imu  = rospy.Publisher("/imu/trim", Imu)
  
  trim_x = rospy.get_param("~trim_x", None)
  trim_y = rospy.get_param("~trim_y", None)  
  trim_z = rospy.get_param("~trim_z", None)
    
  prev_x = 0.0
  prev_y = 0.0
  prev_z = 0.0
  
  rospy.Subscriber("/imu/bias", Imu,  sub_imuCB)
  rospy.spin()
  
  
  
