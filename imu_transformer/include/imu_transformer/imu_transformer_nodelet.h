#ifndef IMU_TRANSFORMER_IMU_TRANSFORMER_NODELET
#define IMU_TRANSFORMER_IMU_TRANSFORMER_NODELET

#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "nodelet/nodelet.h"
#include "message_filters/subscriber.h"
#include "topic_tools/shape_shifter.h"

#include <map>
#include <string>

namespace imu_transformer
{
  typedef sensor_msgs::Imu ImuMsg;
  typedef sensor_msgs::MagneticField MagMsg;
  typedef message_filters::Subscriber<ImuMsg> ImuSubscriber;
//  typedef message_filters::Subscriber<MagMsg> MagSubscriber;
  typedef message_filters::Subscriber<topic_tools::ShapeShifter> MagSubscriber;
  typedef tf2_ros::MessageFilter<ImuMsg> ImuFilter;
  typedef tf2_ros::MessageFilter<MagMsg> MagFilter;

  class ImuTransformerNodelet : public nodelet::Nodelet
  {

  public:
    ImuTransformerNodelet() {};

  private:

    std::string target_frame_;
    std::map <std::string, std::string> source_frame_map_;
    bool convert_ned_to_enu_;

    ros::NodeHandle nh_in_, nh_out_, private_nh_;
    boost::shared_ptr<tf2_ros::Buffer> tf2_;
    boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    ImuSubscriber imu_sub_;
    MagSubscriber mag_sub_;

    boost::shared_ptr<ImuFilter> imu_filter_;
    boost::shared_ptr<MagFilter> mag_filter_;

    ros::Publisher imu_pub_, mag_pub_;

    virtual void onInit();
    void imuCallback(const ImuMsg::ConstPtr &imu_in);
//    void magCallback(const MagMsg::ConstPtr &mag_in);
    void magCallback(const topic_tools::ShapeShifter::ConstPtr &msg);
    void failureCb(tf2_ros::filter_failure_reasons::FilterFailureReason reason);
    void joinImuMsgs(const sensor_msgs::Imu &imu_global, const sensor_msgs::Imu &imu_local, sensor_msgs::Imu &imu_out);

  };

}  // namespace imu_transformer

#endif  // IMU_TRANSFORMER_IMU_TRANSFORMER_NODELET
