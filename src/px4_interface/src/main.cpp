#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "px4_interface/px4_gateway.hpp"
#include "px4_interface/px4_msgs_cache.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto cache = std::make_shared<Px4MsgsCache>();
  auto node = std::make_shared<PX4Gateway>(rclcpp::NodeOptions{}, cache);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
