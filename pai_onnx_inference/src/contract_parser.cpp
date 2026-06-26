#include "pai_onnx_inference/contract_parser.hpp"

#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace pai_onnx_inference {
namespace {

QoS parse_qos(const YAML::Node & n) {
  QoS q;
  if (auto r = n["reliability"]) q.reliability = r.as<std::string>();
  if (auto h = n["history"])     q.history     = h.as<std::string>();
  if (auto d = n["depth"])       q.depth       = d.as<std::int32_t>();
  if (q.depth <= 0) q.depth = 10;
  return q;
}

std::string str_or_empty(const YAML::Node & n) {
  return n ? n.as<std::string>() : std::string{};
}

}  // namespace

Contract parse_contract(const std::string & path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    throw std::runtime_error("contract: cannot load '" + path + "': " + e.what());
  }

  Contract c;
  if (auto r = root["robot_type"]) c.robot_type = r.as<std::string>();
  if (!root["fps"]) {
    throw std::runtime_error("contract: missing required key 'fps'");
  }
  c.fps = root["fps"].as<std::int32_t>();
  if (c.fps <= 0) {
    throw std::runtime_error("contract: fps must be > 0");
  }

  for (const auto & node : root["observations"]) {
    ObservationSpec o;
    o.key      = str_or_empty(node["key"]);
    o.topic    = str_or_empty(node["topic"]);
    o.ros_type = str_or_empty(node["type"]);
    if (auto img = node["image"]; img && img["resize"]) {
      o.image_resize.first  = img["resize"][0].as<std::int32_t>();
      o.image_resize.second = img["resize"][1].as<std::int32_t>();
    }
    if (auto sel = node["selector"]; sel && sel["names"]) {
      o.selector_names = sel["names"].as<std::vector<std::string>>();
    }
    if (auto u = node["unit_conversion"]) o.unit_conversion = u.as<std::string>();
    if (auto a = node["align"]) {
      if (a["strategy"]) o.align_strategy = a["strategy"].as<std::string>();
      if (a["tol_ms"])   o.align_tol_ms   = a["tol_ms"].as<std::int32_t>();
    }
    if (node["qos"]) o.qos = parse_qos(node["qos"]);
    if (o.key.empty() || o.topic.empty()) {
      throw std::runtime_error("contract: observation missing 'key' or 'topic'");
    }
    c.observations.push_back(std::move(o));
  }

  for (const auto & node : root["actions"]) {
    ActionSpec a;
    a.key           = str_or_empty(node["key"]);
    a.publish_topic = str_or_empty(node["publish"]["topic"]);
    a.publish_type  = str_or_empty(node["publish"]["type"]);
    if (auto sel = node["selector"]; sel && sel["names"]) {
      a.selector_names = sel["names"].as<std::vector<std::string>>();
    }
    if (auto u = node["unit_conversion"]) a.unit_conversion = u.as<std::string>();
    if (auto t = node["from_tensor"]; t && t["clamp"]) {
      a.clamp_low  = t["clamp"][0].as<double>();
      a.clamp_high = t["clamp"][1].as<double>();
    }
    if (auto s = node["safety_behavior"]) a.safety_behavior = s.as<std::string>();
    if (node["publish"]["qos"]) a.qos = parse_qos(node["publish"]["qos"]);
    if (a.key.empty() || a.publish_topic.empty()) {
      throw std::runtime_error("contract: action missing 'key' or 'publish.topic'");
    }
    c.actions.push_back(std::move(a));
  }

  return c;
}

}  // namespace pai_onnx_inference
