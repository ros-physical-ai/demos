#include <rclcpp/rclcpp.hpp>
#include "pai_onnx_inference/inference_node.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pai_onnx_inference::InferenceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
