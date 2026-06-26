#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace pai_onnx_inference {

struct QoS {
  std::string reliability;   // "best_effort" | "reliable"
  std::string history;       // "keep_last" | "keep_all"
  std::int32_t depth{10};
};

struct ObservationSpec {
  std::string key;                                  // e.g. "observation.images.wrist"
  std::string topic;                                // e.g. "/wrist_camera/image_raw"
  std::string ros_type;                             // e.g. "sensor_msgs/msg/Image"
  std::pair<std::int32_t, std::int32_t> image_resize{0, 0};  // (h, w); (0,0) if N/A
  std::vector<std::string> selector_names;          // for JointState
  std::string unit_conversion;                      // "rad2deg" | "" (none)
  std::string align_strategy;                       // "hold"
  std::int32_t align_tol_ms{100};
  QoS qos;
};

struct ActionSpec {
  std::string key;                                  // e.g. "action"
  std::string publish_topic;                        // e.g. "/forward_position_controller/commands"
  std::string publish_type;                         // e.g. "std_msgs/msg/Float64MultiArray"
  std::vector<std::string> selector_names;
  std::string unit_conversion;                      // "rad2deg"
  double clamp_low{-3.14159265358979323846};
  double clamp_high{ 3.14159265358979323846};
  std::string safety_behavior;                      // "hold"
  QoS qos;
};

struct Contract {
  std::string robot_type;
  std::int32_t fps{50};
  std::vector<ObservationSpec> observations;
  std::vector<ActionSpec> actions;
};

}  // namespace pai_onnx_inference