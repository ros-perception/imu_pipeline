#ifndef IMU_TRANSFORMER_IMU_TRANSFORMER_HPP
#define IMU_TRANSFORMER_IMU_TRANSFORMER_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "message_filters/subscriber.h"

#include <string>

namespace imu_transformer
{
  typedef sensor_msgs::msg::Imu ImuMsg;
  typedef sensor_msgs::msg::MagneticField MagMsg;
  typedef message_filters::Subscriber<ImuMsg> ImuSubscriber;
  typedef message_filters::Subscriber<MagMsg> MagSubscriber;
  typedef tf2_ros::MessageFilter<ImuMsg> ImuFilter;
  typedef tf2_ros::MessageFilter<MagMsg> MagFilter;

  class ImuTransformer : public rclcpp::Node
  {

  public:
    explicit ImuTransformer(const rclcpp::NodeOptions &);
    ~ImuTransformer();

  private:

    std::string target_frame_;

    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

    ImuSubscriber imu_sub_;
    MagSubscriber mag_sub_;

    std::shared_ptr<ImuFilter> imu_filter_;
    std::shared_ptr<MagFilter> mag_filter_;

    rclcpp::Publisher<ImuMsg>::SharedPtr imu_pub_;
    rclcpp::Publisher<MagMsg>::SharedPtr mag_pub_;

    void imuCallback(const ImuMsg::SharedPtr &imu_in);
    void magCallback(const MagMsg::SharedPtr &msg);
    void failureCb(tf2_ros::filter_failure_reasons::FilterFailureReason reason);

  };

}  // namespace imu_transformer

#endif  // IMU_TRANSFORMER_IMU_TRANSFORMER_HPP
