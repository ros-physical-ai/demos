#include <gtest/gtest.h>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include "pai_onnx_inference/contract_parser.hpp"

namespace fs = std::filesystem;
using pai_onnx_inference::parse_contract;

static fs::path write_yaml(const std::string & body) {
  fs::path p = fs::temp_directory_path() /
    ("contract_" + std::to_string(std::rand()) + ".yaml");
  std::ofstream(p) << body;
  return p;
}

TEST(ContractParser, LoadsSoArm101Contract) {
  const std::string yaml = R"YAML(
robot_type: so_arm101
fps: 50
observations:
  - key: observation.images.wrist
    topic: /wrist_camera/image_raw
    type: sensor_msgs/msg/Image
    image: { resize: [480, 480] }
    align: { strategy: hold, stamp: header, tol_ms: 100 }
    qos: { reliability: best_effort, history: keep_last, depth: 10 }
  - key: observation.images.static
    topic: /static_camera/image_raw
    type: sensor_msgs/msg/Image
    image: { resize: [480, 480] }
    align: { strategy: hold, stamp: header, tol_ms: 100 }
    qos: { reliability: best_effort, history: keep_last, depth: 10 }
  - key: observation.state
    topic: /joint_states
    type: sensor_msgs/msg/JointState
    selector:
      names:
        - position.shoulder_pan_joint
        - position.shoulder_lift_joint
        - position.elbow_flex_joint
        - position.wrist_flex_joint
        - position.wrist_roll_joint
        - position.gripper_joint
    align: { strategy: hold, stamp: header, tol_ms: 100 }
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
    selector:
      names:
        - shoulder_pan.pos
        - shoulder_lift.pos
        - elbow_flex.pos
        - wrist_flex.pos
        - wrist_roll.pos
        - gripper.pos
    unit_conversion: rad2deg
    from_tensor: { clamp: [-3.14159, 3.14159] }
    safety_behavior: hold
recording: { storage: mcap }
)YAML";
  const auto path = write_yaml(yaml);
  auto c = parse_contract(path.string());

  EXPECT_EQ(c.fps, 50);
  ASSERT_EQ(c.observations.size(), 3u);
  EXPECT_EQ(c.observations[0].key, "observation.images.wrist");
  EXPECT_EQ(c.observations[0].image_resize.first, 480);
  EXPECT_EQ(c.observations[0].image_resize.second, 480);
  EXPECT_EQ(c.observations[0].qos.reliability, "best_effort");
  EXPECT_EQ(c.observations[2].unit_conversion, "rad2deg");
  EXPECT_EQ(c.observations[2].selector_names.size(), 6u);
  ASSERT_EQ(c.actions.size(), 1u);
  EXPECT_EQ(c.actions[0].publish_topic, "/forward_position_controller/commands");
  EXPECT_EQ(c.actions[0].selector_names.size(), 6u);
  EXPECT_DOUBLE_EQ(c.actions[0].clamp_low, -3.14159);
  EXPECT_DOUBLE_EQ(c.actions[0].clamp_high,  3.14159);

  fs::remove(path);
}

TEST(ContractParser, RejectsMissingFps) {
  const std::string yaml = "robot_type: so_arm101\nobservations: []\nactions: []\n";
  const auto path = write_yaml(yaml);
  EXPECT_THROW(parse_contract(path.string()), std::runtime_error);
  fs::remove(path);
}

TEST(ContractParser, RejectsNonexistentFile) {
  EXPECT_THROW(parse_contract("/nonexistent/path/to/file.yaml"), std::runtime_error);
}