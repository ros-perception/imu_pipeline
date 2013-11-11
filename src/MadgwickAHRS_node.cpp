/*
# 
# MadgwickAHRS Orientation Estimation Filter
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
*/ 

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

extern "C" {
  #include "MadgwickAHRS/MadgwickAHRS.h"
}

ros::Publisher pub_imu;
ros::Time Prev_msg_stamp;

sensor_msgs::MagneticField msg_mag;

void imu_callback(const sensor_msgs::ImuConstPtr& msg_in){
  sensor_msgs::Imu msg_out(*msg_in);
    
  // Initialize the sample frequency
  if (sampleFreq == 0)
  {
    Prev_msg_stamp = msg_in->header.stamp;
    sampleFreq = 1;
    return;
  }
  
  sampleFreq =  1.0/(msg_in->header.stamp - Prev_msg_stamp).toSec(); 
    
  // Filter Update
  float gx, gy, gz;
  float ax, ay, az;
  float mx, my, mz;
  
  gx = msg_in->angular_velocity.x;
  gy = msg_in->angular_velocity.y;
  gz = msg_in->angular_velocity.z;
  
  ax = msg_in->linear_acceleration.x;
  ay = msg_in->linear_acceleration.y;
  az = msg_in->linear_acceleration.z;
  
  mx = msg_mag.magnetic_field.x;
  my = msg_mag.magnetic_field.y;
  mz = msg_mag.magnetic_field.z;
  msg_mag.magnetic_field.x = 0;
  msg_mag.magnetic_field.y = 0;
  msg_mag.magnetic_field.z = 0;
  
  MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
  
  msg_out.orientation.w = q0;
  msg_out.orientation.x = q1;
  msg_out.orientation.y = q2;
  msg_out.orientation.z = q3;
  
  pub_imu.publish(msg_out);

  Prev_msg_stamp = msg_in->header.stamp;
}


void mag_callback(const sensor_msgs::MagneticFieldConstPtr& msg_in){
  msg_mag.header = msg_in->header;
  msg_mag.magnetic_field = msg_in->magnetic_field;
}


int main(int argc, char **argv){
  ros::init(argc, argv, "MadgwickAHRS");
  ros::NodeHandle n;
  ros::NodeHandle n_("~");
  
  sampleFreq = 0;
  
  // Init mag field
  msg_mag.magnetic_field.x = 0;
  msg_mag.magnetic_field.y = 0;
  msg_mag.magnetic_field.z = 0;
  
  pub_imu = n.advertise<sensor_msgs::Imu>("/imu/orient", 10);
  
  ros::Subscriber sub_imu = n.subscribe("/imu/trim", 10, imu_callback);  
  ros::Subscriber sub_mag = n.subscribe("/android/magnetic_field", 10, mag_callback);  
  
  ros::spin();
  return 0;
}


