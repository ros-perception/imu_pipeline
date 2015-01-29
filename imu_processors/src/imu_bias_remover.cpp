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
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>

ros::Publisher pub_;
ros::Publisher bias_pub_;

bool use_cmd_vel_;
bool use_odom_;

bool twist_is_zero_;
bool odom_is_zero_;

double cmd_vel_threshold_;
double odom_threshold_;

// Implement an exponentially weighted moving average to calculate bias
double accumulator_alpha_;
geometry_msgs::Vector3 angular_velocity_accumulator;

// Returns true if |val1| < val2
bool abslt(const double& val1, const double& val2){
  return std::abs(val1) < val2;
}

double accumulator_update(const double& alpha, const double& avg, const double& meas){
  return alpha * meas + (1.0 - alpha) * avg;
}

void cmd_vel_callback(const geometry_msgs::TwistConstPtr& msg){
  if(use_cmd_vel_){
    if(abslt(msg->linear.x, cmd_vel_threshold_) &&
       abslt(msg->linear.y, cmd_vel_threshold_) &&
       abslt(msg->linear.z, cmd_vel_threshold_) &&
       abslt(msg->angular.x, cmd_vel_threshold_) &&
       abslt(msg->angular.y, cmd_vel_threshold_) &&
       abslt(msg->angular.z, cmd_vel_threshold_)){
      twist_is_zero_ = true;
      return;
    }
  }
  twist_is_zero_ = false;
}

void odom_callback(const nav_msgs::OdometryConstPtr& msg){
  if(use_odom_){
    if(abslt(msg->twist.twist.linear.x, odom_threshold_) &&
       abslt(msg->twist.twist.linear.y, odom_threshold_) &&
       abslt(msg->twist.twist.linear.z, odom_threshold_) &&
       abslt(msg->twist.twist.angular.x, odom_threshold_) &&
       abslt(msg->twist.twist.angular.y, odom_threshold_) &&
       abslt(msg->twist.twist.angular.z, odom_threshold_)){
      odom_is_zero_ = true;
      return;
    }
  }
  odom_is_zero_ = false;
}

void imu_callback(const sensor_msgs::ImuConstPtr& msg){
  sensor_msgs::ImuPtr imu(new sensor_msgs::Imu(*msg));

  if(twist_is_zero_ || odom_is_zero_){ // Update bias, set outputs to 0
    angular_velocity_accumulator.x = accumulator_update(accumulator_alpha_, angular_velocity_accumulator.x, msg->angular_velocity.x);
    angular_velocity_accumulator.y = accumulator_update(accumulator_alpha_, angular_velocity_accumulator.y, msg->angular_velocity.y);
    angular_velocity_accumulator.z = accumulator_update(accumulator_alpha_, angular_velocity_accumulator.z, msg->angular_velocity.z);
    imu->angular_velocity.x = 0.0;
    imu->angular_velocity.y = 0.0;
    imu->angular_velocity.z = 0.0;
  } else { // Modify outputs by bias
    imu->angular_velocity.x -= angular_velocity_accumulator.x;
    imu->angular_velocity.y -= angular_velocity_accumulator.y;
    imu->angular_velocity.z -= angular_velocity_accumulator.z;
  }

  // Publish transformed message
	pub_.publish(imu);

  // Publish bias information
  geometry_msgs::Vector3StampedPtr bias(new geometry_msgs::Vector3Stamped());
  bias->header = imu->header;
  bias->vector = angular_velocity_accumulator;
  bias_pub_.publish(bias);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "imu_bias_remover");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  // Initialize
  twist_is_zero_ = false;
  odom_is_zero_ = false;
  angular_velocity_accumulator.x = 0.0;
  angular_velocity_accumulator.y = 0.0;
  angular_velocity_accumulator.z = 0.0;

  // Get parameters
  pnh.param<bool>("use_cmd_vel", use_cmd_vel_, false);
  pnh.param<bool>("use_odom", use_odom_, false);
  pnh.param<double>("accumulator_alpha", accumulator_alpha_, 0.01);
  
  ros::Subscriber cmd_sub;
  if(use_cmd_vel_){
    cmd_sub = n.subscribe("cmd_vel", 10, cmd_vel_callback);
  }
  ros::Subscriber odom_sub;
  if(use_odom_){
    odom_sub = n.subscribe("odom", 10, odom_callback);
  }

  pnh.param<double>("cmd_vel_threshold", cmd_vel_threshold_, 0.001);
  pnh.param<double>("odom_threshold", odom_threshold_, 0.001);
  
  // Create publisher
  pub_ = n.advertise<sensor_msgs::Imu>("imu_biased", 10);
  bias_pub_ = n.advertise<geometry_msgs::Vector3Stamped>("bias", 10);

  // Imu Subscriber
  ros::Subscriber sub = n.subscribe("imu", 100, imu_callback);  
  
  ros::spin();

  return 0;
}
