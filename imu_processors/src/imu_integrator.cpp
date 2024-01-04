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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace imu_processors
{

class ImuIntegrator : public rclcpp::Node
{
public:
  explicit ImuIntegrator(const rclcpp::NodeOptions & options)
  : rclcpp::Node("imu_integrator", options)
  {
    // Get parameters
    min_angular_velocity_ = this->declare_parameter<double>("min_angular_velocity", 0.0);
    static_dt_ = this->declare_parameter<double>("static_dt", 0.0);
    imu_.orientation.w = 1.0;

    // Create publisher
    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_integrated", 10);

    // Subscriber
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SystemDefaultsQoS(),
      std::bind(&ImuIntegrator::callback, this, std::placeholders::_1));
  }

private:
  void callback(const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
  {
    rclcpp::Time msg_time(msg->header.stamp);
    rclcpp::Time imu_time(imu_.header.stamp);
    double dt = msg_time.seconds() - imu_time.seconds();
    if (imu_time.seconds() == 0)
    {
      dt = 0.0;
    }
    if (static_dt_ > 0.0001)
    {
      dt = static_dt_;
    }
    imu_.header = msg->header;
    imu_.angular_velocity = msg->angular_velocity;
    imu_.angular_velocity_covariance = msg->angular_velocity_covariance;
    imu_.linear_acceleration = msg->linear_acceleration;
    imu_.linear_acceleration_covariance = msg->linear_acceleration_covariance;

    double diff = pow(imu_.orientation.w, 2.0) - pow(imu_.orientation.z, 2.0);
    double mult = 2.0 * imu_.orientation.w * imu_.orientation.z;
    double theta = atan2(mult, diff);
    if (std::abs(imu_.angular_velocity.z) > min_angular_velocity_)
    {
      theta = theta + imu_.angular_velocity.z * dt;
    }

    // Fill in quaternion
    imu_.orientation.z = sin(theta / 2.0);
    imu_.orientation.w = cos(theta / 2.0);

    // Publish transformed message
    pub_->publish(imu_);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  sensor_msgs::msg::Imu imu_;
  double min_angular_velocity_;
  double static_dt_;
};

}  // namespace imu_processors

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(imu_processors::ImuIntegrator)
