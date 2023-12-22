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


#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>

namespace imu_processors
{

// Returns true if |val1| < val2
bool abslt(const double& val1, const double& val2)
{
  return std::abs(val1) < val2;
}

double accumulator_update(const double& alpha, const double& avg, const double& meas)
{
  return alpha * meas + (1.0 - alpha) * avg;
}

class ImuBiasRemover : public rclcpp::Node
{
public:
  explicit ImuBiasRemover(const rclcpp::NodeOptions & options)
  : rclcpp::Node("imu_bias_remover", options)
  {
    twist_is_zero_ = false;
    odom_is_zero_ = false;
    accumulator_.x = 0.0;
    accumulator_.y = 0.0;
    accumulator_.z = 0.0;

    // Get parameters
    use_cmd_vel_ = this->declare_parameter<bool>("use_cmd_vel", false);
    use_odom_ = this->declare_parameter<bool>("use_odom", false);
    alpha_ = this->declare_parameter<double>("accumulator_alpha", 0.01);

    if (use_cmd_vel_)
    {
      RCLCPP_INFO(rclcpp::get_logger("imu_bias_remover"), "Using cmd_vel");
      cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::SystemDefaultsQoS(),
        std::bind(&ImuBiasRemover::cmd_vel_callback, this, std::placeholders::_1));
    }
    if (use_odom_)
    {
      RCLCPP_INFO(rclcpp::get_logger("imu_bias_remover"), "Using odom");
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SystemDefaultsQoS(),
        std::bind(&ImuBiasRemover::odom_callback, this, std::placeholders::_1));
    }

    cmd_vel_threshold_ = this->declare_parameter<double>("cmd_vel_threshold", 0.001);
    odom_threshold_ = this->declare_parameter<double>("odom_threshold", 0.001);

    // Create publisher
    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_biased", 10);
    bias_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("bias", 10);

    // Imu Subscriber
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SystemDefaultsQoS(),
      std::bind(&ImuBiasRemover::imu_callback, this, std::placeholders::_1));
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr & msg)
  {
    if (abslt(msg->linear.x, cmd_vel_threshold_) &&
        abslt(msg->linear.y, cmd_vel_threshold_) &&
        abslt(msg->linear.z, cmd_vel_threshold_) &&
        abslt(msg->angular.x, cmd_vel_threshold_) &&
        abslt(msg->angular.y, cmd_vel_threshold_) &&
        abslt(msg->angular.z, cmd_vel_threshold_))
    {
      twist_is_zero_ = true;
      return;
    }
    twist_is_zero_ = false;
  }

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
  {
    if (abslt(msg->twist.twist.linear.x, odom_threshold_) &&
        abslt(msg->twist.twist.linear.y, odom_threshold_) &&
        abslt(msg->twist.twist.linear.z, odom_threshold_) &&
        abslt(msg->twist.twist.angular.x, odom_threshold_) &&
        abslt(msg->twist.twist.angular.y, odom_threshold_) &&
        abslt(msg->twist.twist.angular.z, odom_threshold_))
    {
      odom_is_zero_ = true;
      return;
    }
    odom_is_zero_ = false;
  }

  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
  {
    sensor_msgs::msg::Imu imu(*msg);

    if (twist_is_zero_ || odom_is_zero_)
    {
      // Update bias, set outputs to 0
      accumulator_.x = accumulator_update(alpha_, accumulator_.x, msg->angular_velocity.x);
      accumulator_.y = accumulator_update(alpha_, accumulator_.y, msg->angular_velocity.y);
      accumulator_.z = accumulator_update(alpha_, accumulator_.z, msg->angular_velocity.z);
      imu.angular_velocity.x = 0.0;
      imu.angular_velocity.y = 0.0;
      imu.angular_velocity.z = 0.0;
    } else {
      // Modify outputs by bias
      imu.angular_velocity.x -= accumulator_.x;
      imu.angular_velocity.y -= accumulator_.y;
      imu.angular_velocity.z -= accumulator_.z;
    }

    // Publish transformed message
    pub_->publish(imu);

    // Publish bias information
    geometry_msgs::msg::Vector3Stamped bias;
    bias.header = imu.header;
    bias.vector = accumulator_;
    bias_pub_->publish(bias);
  }

private:
  bool twist_is_zero_;
  bool odom_is_zero_;

  bool use_cmd_vel_;
  bool use_odom_;

  double cmd_vel_threshold_;
  double odom_threshold_;

  // Implement an exponentially weighted moving average to calculate bias
  geometry_msgs::msg::Vector3 accumulator_;
  double alpha_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr bias_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

}  // namespace imu_processors

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(imu_processors::ImuBiasRemover)
