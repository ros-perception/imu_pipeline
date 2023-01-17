#include <rclcpp/rclcpp.hpp>
#include <imu_transformer/imu_transformer.hpp>

#include <memory>

int main(int argc, char **argv){
  rclcpp::init(argc, argv);

  const rclcpp::NodeOptions options;

  rclcpp::spin(std::make_shared<imu_transformer::ImuTransformer>(options));
  rclcpp::shutdown();
  return 0;
}
