#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "px4_interface/vslam_odom_bridge.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VslamOdomBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
