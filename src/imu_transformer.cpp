/*
 * Copyright (c) 2012, Willow Garage, Inc.
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
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>

ros::Publisher pub_;
boost::shared_ptr<tf::TransformListener> listener_;
std::string target_frame_id_;

void transformCovariance(const boost::array<double, 9>& in, boost::array<double, 9>& out, const tf::StampedTransform& transform){
	tf::Matrix3x3 cov_in(in[0], in[1], in[2],
    				   in[3], in[4], in[5],
    				   in[6], in[7], in[8]);

	tf::Matrix3x3 rot(transform.getRotation());
    tf::Matrix3x3 cov_out = rot*cov_in*(rot.transpose());
    for(size_t i = 0; i < 3; i++){
    	tf::Vector3 row = cov_out.getRow(i);
    	out[3*i] = row.getX();
    	out[3*i+1] = row.getY();
    	out[3*i+2] = row.getZ();
    }
}

void imu_callback(const sensor_msgs::ImuConstPtr& msg){
	// Create output message
	sensor_msgs::ImuPtr msg_out(new sensor_msgs::Imu());
	msg_out->header = msg->header;
	msg_out->header.frame_id = target_frame_id_;

	// Get transform
	tf::StampedTransform transform;
    
    try {
      listener_->lookupTransform(target_frame_id_, msg->header.frame_id, msg->header.stamp, transform);
      transform.setOrigin(tf::Vector3(0,0,0)); // Clear translation component so we can use TF's transform operator directly.
    } catch (tf::TransformException &ex) {
      ROS_ERROR("Could not transform imu from frame %s to %s: %s", msg->header.frame_id.c_str(), target_frame_id_.c_str(), ex.what());
      return;
    }

    // Orienation
    if(msg->orientation_covariance[0] < 0){ // Invalid covariance.  Skip these elements.
    	msg_out->orientation = msg->orientation;
    	msg_out->orientation_covariance = msg->orientation_covariance;
	} else {
	    // Transform orientation
	    tf::Quaternion quat;
	    tf::quaternionMsgToTF(msg->orientation, quat);
	    tf::quaternionTFToMsg(transform*quat, msg_out->orientation);

	    // Transform orientation_covariance
	    transformCovariance(msg->orientation_covariance, msg_out->orientation_covariance, transform);
    }

    // Angular velocity
    if(msg->angular_velocity_covariance[0] < 0){ // Invalid covariance.  Skip these elements.
    	msg_out->angular_velocity = msg->angular_velocity;
    	msg_out->angular_velocity_covariance = msg->angular_velocity_covariance;
	} else {
	    // Transform angular_velocity
	    tf::Vector3 gyro;
	    tf::vector3MsgToTF(msg->angular_velocity, gyro);
	    tf::vector3TFToMsg(transform*gyro, msg_out->angular_velocity);

	    // Transform angular_velocity_covariance
	    transformCovariance(msg->angular_velocity_covariance, msg_out->angular_velocity_covariance, transform);
    }

    // Linear acceleration
    if(msg->linear_acceleration_covariance[0] < 0){ // Invalid covariance.  Skip these elements.
    	msg_out->linear_acceleration = msg->linear_acceleration;
    	msg_out->linear_acceleration_covariance = msg->linear_acceleration_covariance;
	} else {
	    // Transform linear_acceleration
	    tf::Vector3 accel;
	    tf::vector3MsgToTF(msg->linear_acceleration, accel);
	    tf::vector3TFToMsg(transform*accel, msg_out->linear_acceleration);

	    // Transform linear_acceleration_covariance
	    transformCovariance(msg->linear_acceleration_covariance, msg_out->linear_acceleration_covariance, transform);
    }

    // Publish transformed message
	pub_.publish(msg_out);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "imu_transformer");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  // Get parameters
  pnh.param<std::string>("target_frame_id", target_frame_id_, "base_link");
  int message_filter_queue_size;
  pnh.param<int>("queue_size", message_filter_queue_size, 1000);
  
  // Create publisher
  pub_ = n.advertise<sensor_msgs::Imu>("imu_transformed", 2);

  // Create tf listener
  listener_.reset(new tf::TransformListener());

  // Create tf message filter to automatically handle queuing messages until tf is ready
  message_filters::Subscriber<sensor_msgs::Imu> sub;
  sub.subscribe(n, "imu", 2);
  tf::MessageFilter<sensor_msgs::Imu>* tf_filter;
  tf_filter = new tf::MessageFilter<sensor_msgs::Imu>(sub, *listener_, target_frame_id_, message_filter_queue_size);
  tf_filter->registerCallback(boost::bind(&imu_callback, _1));
  
  ros::spin();

  return 0;
}