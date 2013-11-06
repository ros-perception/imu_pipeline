#!/usr/bin/env python
# 
# Orientation to TF - Generates an parent frame from the IMU message
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
 
  
# Filter the messages   
def sub_imuCB(msg_in): 
  global br
  
  br.sendTransform((0, 0, 0),
                   (msg_in.orientation.x, 
                    msg_in.orientation.y, 
                    msg_in.orientation.z, 
                    msg_in.orientation.w ),
                   rospy.Time.now(),
                   "imu",     # child
                   "world"    # parent
                  )


if __name__ == '__main__':
  global br
  
  rospy.init_node('orient_tf')
  
  br = tf.TransformBroadcaster() 
  
  rospy.Subscriber("/imu/orient", Imu,  sub_imuCB)
  rospy.spin()
  
  
  
