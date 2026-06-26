#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "pai_onnx_inference/inference_node.hpp"
#include "rosetta_interfaces/action/run_policy.hpp"

using std::chrono::milliseconds;
namespace fs = std::filesystem;

namespace {

std::string write_contract_yaml() {
  const std::string body = R"YAML(
robot_type: so_arm101
fps: 50
observations:
  - key: observation.images.wrist
    topic: /wrist_camera/image_raw
    type: sensor_msgs/msg/Image
    image: { resize: [4, 4] }
    align: { strategy: hold, stamp: header, tol_ms: 200 }
    qos: { reliability: best_effort, history: keep_last, depth: 10 }
  - key: observation.state
    topic: /joint_states
    type: sensor_msgs/msg/JointState
    selector:
      names: [a, b, c, d, e, f]
    align: { strategy: hold, stamp: header, tol_ms: 200 }
    qos: { reliability: reliable, history: keep_last, depth: 50 }
    unit_conversion: rad2deg
actions:
  - key: action
    publish:
      topic: /forward_position_controller/commands
      type: std_msgs/msg/Float64MultiArray
      layout: flat
      qos: { reliability: reliable, history: keep_last, depth: 10 }
      strategy: { mode: nearest, tolerance_ms: 100 }
    selector: { names: [a, b, c, d, e, f] }
    unit_conversion: rad2deg
    from_tensor: { clamp: [-3.14159, 3.14159] }
    safety_behavior: hold
)YAML";
  const std::string path = "/tmp/pai_onnx_inference_test_contract.yaml";
  std::ofstream(path) << body;
  return path;
}

sensor_msgs::msg::Image::SharedPtr fake_image(const std::string & frame_id) {
  auto msg = std::make_shared<sensor_msgs::msg::Image>();
  msg->header.frame_id = frame_id;
  msg->header.stamp = rclcpp::Clock().now();
  msg->height = 2; msg->width = 2; msg->encoding = "bgr8";
  msg->step = 6; msg->data = {1,2,3,4,5,6,7,8,9,10,11,12};
  return msg;
}

sensor_msgs::msg::JointState::SharedPtr fake_joints() {
  auto msg = std::make_shared<sensor_msgs::msg::JointState>();
  msg->header.stamp = rclcpp::Clock().now();
  msg->name = {"a", "b", "c", "d", "e", "f"};
  msg->position = {0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
  return msg;
}

}  // namespace

class InferenceNodeFixture : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }
  void TearDown() override {
    if (rclcpp::ok()) rclcpp::shutdown();
  }
};

TEST_F(InferenceNodeFixture, RunsAndPublishesOnGoal) {
  const std::string contract = write_contract_yaml();
  const std::string model = std::string(PAI_ONNX_FIXTURE_DIR) + "/identity_policy.onnx";

  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    rclcpp::Parameter("contract_path", contract),
    rclcpp::Parameter("model_path", model),
    rclcpp::Parameter("ep_requested", "cpu"),
    rclcpp::Parameter("ep_policy", "strict"),
    rclcpp::Parameter("use_sim_time", false),
  });
  auto node = std::make_shared<pai_onnx_inference::InferenceNode>(opts);

  std::atomic<bool> received{false};
  auto sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/forward_position_controller/commands", 10,
      [&](const std_msgs::msg::Float64MultiArray::SharedPtr) { received.store(true); });

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread spin_thread([&] { executor->spin(); });

  // Wait for the action server to come up.
  auto client = rclcpp_action::create_client<rosetta_interfaces::action::RunPolicy>(
      node, "/run_policy");
  ASSERT_TRUE(client->wait_for_action_server(milliseconds(2000)));

  rosetta_interfaces::action::RunPolicy::Goal goal;
  goal.prompt = "test";
  client->async_send_goal(goal);

  // Give it a beat to start accepting.
  std::this_thread::sleep_for(milliseconds(200));

  // Drive publishers from the executor's perspective: use the same node so they
  // share the executor's clock.
  auto pub_img = node->create_publisher<sensor_msgs::msg::Image>("/wrist_camera/image_raw", 10);
  auto pub_js  = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 50);

  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(3)) {
    pub_img->publish(*fake_image("wrist"));
    pub_js->publish(*fake_joints());
    std::this_thread::sleep_for(milliseconds(50));
  }

  EXPECT_TRUE(received.load()) << "no actions published in 3s window";

  executor->cancel();
  spin_thread.join();

  std::remove(contract.c_str());
}