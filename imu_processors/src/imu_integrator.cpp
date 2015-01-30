/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* 
 * Author: Chad Rockey
 */

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>

ros::Publisher pub_;
sensor_msgs::ImuPtr imu_;
double min_angular_velocity_;
double static_dt_;

void imu_callback(const sensor_msgs::ImuConstPtr& msg){
  double dt = msg->header.stamp.toSec() - imu_->header.stamp.toSec();
  if(imu_->header.stamp.sec == 0){
    dt = 0.0;
  }
  if(static_dt_ > 0.0001){
    dt = static_dt_;
  }
	imu_->header = msg->header;
  imu_->angular_velocity = msg->angular_velocity;
  imu_->angular_velocity_covariance = msg->angular_velocity_covariance;
  imu_->linear_acceleration = msg->linear_acceleration;
  imu_->linear_acceleration_covariance = msg->linear_acceleration_covariance;

  double diff = pow(imu_->orientation.w, 2.0)-pow(imu_->orientation.z, 2.0);
  double mult = 2.0*imu_->orientation.w*imu_->orientation.z;
  double theta = atan2(mult, diff);
  if(std::abs(imu_->angular_velocity.z) > min_angular_velocity_){
    theta = theta + imu_->angular_velocity.z*dt;
  }
  
  // Fill in quaternion
  imu_->orientation.z = sin(theta/2.0);
  imu_->orientation.w = cos(theta/2.0);

  // Publish transformed message
	pub_.publish(imu_);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "imu_integrator");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  // Get parameters
  pnh.param<double>("min_angular_velocity", min_angular_velocity_, 0.0);
  pnh.param<double>("static_dt", static_dt_, 0.0);
  imu_.reset(new sensor_msgs::Imu());
  imu_->orientation.w = 1.0;
  
  // Create publisher
  pub_ = n.advertise<sensor_msgs::Imu>("imu_integrated", 10);

  // Subscriber
  ros::Subscriber sub = n.subscribe("imu", 100, imu_callback);  
  
  ros::spin();

  return 0;
}