#include <imu_transformer/imu_transformer.hpp>

//#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
//Remove this header when https://github.com/ros/geometry_experimental/pull/78 is released
#include "imu_transformer/tf2_sensor_msgs.h"

namespace imu_transformer
{

  ImuTransformer::ImuTransformer(const rclcpp::NodeOptions & options)
  : Node("imu_transformer", options){

    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");

    imu_pub_ = this->create_publisher<ImuMsg>("imu_out", 10);
    mag_pub_ = this->create_publisher<MagMsg>("mag_out", 10);

    imu_sub_.subscribe(this, "imu_in");

    std::chrono::duration<int> buffer_timeout(1);

    imu_filter_ = std::make_shared<ImuFilter>(imu_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(), this->get_node_clock_interface(), buffer_timeout);
    imu_filter_->registerCallback(&ImuTransformer::imuCallback, this);
    // function deactivated in foxy
    //imu_filter_->registerFailureCallback&ImuTransformer::failureCb, this);

    mag_sub_.subscribe(this, "mag_in");
    mag_filter_ = std::make_shared<MagFilter>(mag_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(), this->get_node_clock_interface(), buffer_timeout);
    mag_filter_->registerCallback(&ImuTransformer::magCallback, this);
    // function deactivated in foxy
    //mag_filter_->registerFailureCallback&ImuTransformer::failureCb, this);
  }

  void ImuTransformer::imuCallback(const ImuMsg::SharedPtr &imu_in)
  {
    try
    {
      ImuMsg imu_out;
      tf2_buffer_->transform(*imu_in, imu_out, target_frame_);
      imu_pub_->publish(imu_out);
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_WARN(
           this->get_logger(), "IMU Transform failure %s\n", ex.what());
    }
  }

  void ImuTransformer::magCallback(const MagMsg::SharedPtr &mag_in)
  {
    std::string error;
    try
    {
      MagMsg mag_out;
      tf2_buffer_->transform(*mag_in, mag_out, target_frame_);
      mag_pub_->publish(mag_out);
      return;
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_WARN(
           this->get_logger(), "IMU Transform failure %s\n", ex.what());
    }

  }

  //void ImuTransformer::failureCb(tf2_ros::filter_failure_reasons::FilterFailureReason reason)
  //{
        //RCLCPP_WARN(
           //this->get_logger(), "Can't transform incoming IMU data to " << target_frame_ << " " << 
        //reason);
  //}
  
  ImuTransformer::~ImuTransformer() {}

}

//PLUGINLIB_EXPORT_CLASS(imu_transformer::ImuTransformerNodelet, nodelet::Nodelet)
