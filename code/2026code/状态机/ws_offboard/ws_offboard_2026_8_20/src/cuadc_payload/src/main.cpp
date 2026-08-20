#include <memory>

#include "cuadc_payload/payload_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cuadc_payload::PayloadNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
