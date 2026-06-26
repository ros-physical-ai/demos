#include "pai_onnx_inference/inference_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "pai_onnx_inference/contract_parser.hpp"

namespace pai_onnx_inference {

namespace {

rclcpp::QoS qos_from_spec(const QoS & q) {
  rmw_qos_reliability_policy_t rel =
      (q.reliability == "best_effort") ? RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
                                       : RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  rmw_qos_history_policy_t hist =
      (q.history == "keep_all") ? RMW_QOS_POLICY_HISTORY_KEEP_ALL
                                : RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  return rclcpp::QoS(rclcpp::QoSInitialization(hist, q.depth)).reliability(rel);
}

}  // namespace

InferenceNode::InferenceNode(const rclcpp::NodeOptions & opts)
    : rclcpp::Node("pai_onnx_inference", opts),
      aligned_(100 /* overwritten after contract load */) {
  declare_parameter("contract_path", "");
  declare_parameter("model_path", "");
  declare_parameter("ep_requested", "auto");
  declare_parameter("ep_policy", "fallback");
  declare_parameter("observation_timeout_s", 1.0);
  declare_parameter("drop_warn_threshold", 5);
  declare_parameter("onnx_error_policy", "log_and_drop");
  declare_parameter("sim_time_watchdog_s", 2.0);

  contract_path_         = get_parameter("contract_path").as_string();
  model_path_            = get_parameter("model_path").as_string();
  ep_requested_          = get_parameter("ep_requested").as_string();
  ep_policy_             = get_parameter("ep_policy").as_string();
  observation_timeout_s_ = get_parameter("observation_timeout_s").as_double();
  drop_warn_threshold_   = get_parameter("drop_warn_threshold").as_int();
  onnx_error_policy_     = get_parameter("onnx_error_policy").as_string();
  sim_time_watchdog_s_   = get_parameter("sim_time_watchdog_s").as_double();

  if (contract_path_.empty()) throw std::runtime_error("contract_path is required");
  if (model_path_.empty())    throw std::runtime_error("model_path is required");

  contract_ = parse_contract(contract_path_);
  aligned_  = AlignedBuffer(contract_.observations.empty() ? 100
                                                           : contract_.observations[0].align_tol_ms);
  assembler_ = std::make_unique<ObservationAssembler>(contract_);
  runner_    = std::make_unique<OnnxRunner>();
  runner_->load(model_path_, ep_requested_, ep_policy_);

  for (const auto & obs : contract_.observations) {
    auto qos = qos_from_spec(obs.qos);
    if (obs.key == "observation.images.wrist") {
      sub_wrist_ = create_subscription<sensor_msgs::msg::Image>(
          obs.topic, qos,
          [this](const sensor_msgs::msg::Image::SharedPtr m) { on_image_wrist(m); });
    } else if (obs.key == "observation.images.static") {
      sub_static_ = create_subscription<sensor_msgs::msg::Image>(
          obs.topic, qos,
          [this](const sensor_msgs::msg::Image::SharedPtr m) { on_image_static(m); });
    } else if (obs.key == "observation.state") {
      sub_states_ = create_subscription<sensor_msgs::msg::JointState>(
          obs.topic, qos,
          [this](const sensor_msgs::msg::JointState::SharedPtr m) { on_joint_states(m); });
    }
  }

  if (contract_.actions.empty()) {
    throw std::runtime_error("contract: no actions defined");
  }
  const auto & act = contract_.actions.front();
  pub_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      act.publish_topic, qos_from_spec(act.qos));

  action_server_ = rclcpp_action::create_server<RunPolicy>(
      this, "/run_policy",
      [this](auto && a, auto && b) { return handle_goal(a, b); },
      [this](auto && a) { return handle_cancel(a); },
      [this](auto && a) { handle_accepted(a); });

  const double period_s = 1.0 / static_cast<double>(contract_.fps);
  tick_timer_ = create_wall_timer(std::chrono::duration<double>(period_s),
                                  [this]() { on_tick(); });
  feedback_timer_ = create_wall_timer(std::chrono::milliseconds(500),
                                       [this]() { publish_feedback(); });

  inference_thread_ = std::thread([this]() { inference_loop(); });

  RCLCPP_INFO(get_logger(),
              "pai_onnx_inference ready: model=%s contract=%s ep=%s fps=%d",
              model_path_.c_str(), contract_path_.c_str(),
              runner_->active_ep().c_str(), contract_.fps);
}

InferenceNode::~InferenceNode() {
  inference_should_exit_.store(true);
  inference_cv_.notify_all();
  if (inference_thread_.joinable()) inference_thread_.join();
  tick_timer_.reset();
  feedback_timer_.reset();
  action_server_.reset();
  sub_wrist_.reset();
  sub_static_.reset();
  sub_states_.reset();
  pub_cmd_.reset();
}

namespace {
inline int64_t stamp_ns(const builtin_interfaces::msg::Time & t) {
  return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
}
}  // namespace

void InferenceNode::on_image_wrist(const sensor_msgs::msg::Image::SharedPtr msg) {
  std::vector<uint8_t> bytes(msg->data.begin(), msg->data.end());
  {
    std::lock_guard<std::mutex> lk(encoding_mu_);
    last_image_encoding_["observation.images.wrist"] = msg->encoding;
    last_image_dims_["observation.images.wrist"] = {msg->height, msg->width};
  }
  aligned_.push("observation.images.wrist", stamp_ns(msg->header.stamp), Payload{std::move(bytes)});
}

void InferenceNode::on_image_static(const sensor_msgs::msg::Image::SharedPtr msg) {
  std::vector<uint8_t> bytes(msg->data.begin(), msg->data.end());
  {
    std::lock_guard<std::mutex> lk(encoding_mu_);
    last_image_encoding_["observation.images.static"] = msg->encoding;
    last_image_dims_["observation.images.static"] = {msg->height, msg->width};
  }
  aligned_.push("observation.images.static", stamp_ns(msg->header.stamp), Payload{std::move(bytes)});
}

void InferenceNode::on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg) {
  // Find the observation.state spec — it's the one with selector_names populated.
  std::vector<std::string> sel;
  for (const auto & obs : contract_.observations) {
    if (obs.key == "observation.state") { sel = obs.selector_names; break; }
  }
  if (sel.empty()) return;
  std::vector<double> values(sel.size(), 0.0);
  for (std::size_t i = 0; i < sel.size(); ++i) {
    auto it = std::find(msg->name.begin(), msg->name.end(), sel[i]);
    if (it == msg->name.end()) continue;
    const auto idx = static_cast<std::size_t>(it - msg->name.begin());
    if (idx < msg->position.size()) values[i] = msg->position[idx];
  }
  aligned_.push("observation.state", stamp_ns(msg->header.stamp), Payload{std::move(values)});
}

rclcpp_action::GoalResponse InferenceNode::handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const RunPolicy::Goal>) {
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse InferenceNode::handle_cancel(
    const std::shared_ptr<GoalHandle>) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void InferenceNode::handle_accepted(const std::shared_ptr<GoalHandle> gh) {
  if (active_goal_) {
    auto prev = std::make_shared<RunPolicy::Result>();
    prev->success = false;
    prev->message = "superseded by new goal";
    active_goal_->abort(prev);
  }
  active_goal_ = gh;
  published_actions_.store(0);
  chunk_queue_.clear();
  goal_active_.store(true);
  // Spawn a waiter thread that finishes the goal when cancelled/completed.
  std::thread([this, gh]() {
    while (rclcpp::ok() && goal_active_.load() && gh->is_active()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (rclcpp::ok() && gh->is_active()) {
      auto res = std::make_shared<RunPolicy::Result>();
      res->success = true;
      res->message = "cancelled by client";
      gh->succeed(res);
    }
  }).detach();
}

void InferenceNode::on_tick() {
  const auto now = this->now();
  const int64_t now_ns = now.nanoseconds();

  // Publish: pop one action off the chunk queue if available.
  if (auto a = chunk_queue_.pop_front()) {
    const auto & act = contract_.actions.front();
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(act.selector_names.size());
    const double kDegToRad = 0.017453292519943295;
    for (std::size_t i = 0; i < a->values.size() && i < msg.data.size(); ++i) {
      double v = a->values[i] * kDegToRad;
      if (v < act.clamp_low)  v = act.clamp_low;
      if (v > act.clamp_high) v = act.clamp_high;
      msg.data[i] = v;
    }
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "joint";
    msg.layout.dim[0].size = static_cast<std::uint32_t>(msg.data.size());
    msg.layout.dim[0].stride = 1;
    pub_cmd_->publish(msg);
    published_actions_.fetch_add(1);
  }

  // Don't request new inferences unless the action goal is active.
  if (!goal_active_.load()) return;

  // Build a fresh aligned observation frame at `now`.
  auto frame = aligned_.pop_at(now_ns);
  if (!frame) return;

  // Cache lookup: per-image-key encoding and dims.
  std::string wrist_enc, static_enc;
  std::pair<int32_t, int32_t> wrist_dims{0, 0}, static_dims{0, 0};
  {
    std::lock_guard<std::mutex> lk(encoding_mu_);
    auto we = last_image_encoding_.find("observation.images.wrist");
    if (we != last_image_encoding_.end()) wrist_enc = we->second;
    auto se = last_image_encoding_.find("observation.images.static");
    if (se != last_image_encoding_.end()) static_enc = se->second;
    auto wd = last_image_dims_.find("observation.images.wrist");
    if (wd != last_image_dims_.end()) wrist_dims = wd->second;
    auto sd = last_image_dims_.find("observation.images.static");
    if (sd != last_image_dims_.end()) static_dims = sd->second;
  }

  // The assembler uses a single src_encoding/dim per call. To support two
  // images with different encodings, call it once per key with a frame that
  // contains only that key. The current assembler overwrites any prior output
  // for that key — to keep multiple keys' outputs we call once with all keys
  // present, then re-call for keys with different encodings using a stripped
  // frame. For the MVP we call twice when encodings differ; this is wasteful
  // but correct. (TODO: extend assembler to accept per-key encoding map.)
  OnnxRunner::FloatMap inputs;
  try {
    const std::string & primary_enc = wrist_enc.empty() ? static_enc : wrist_enc;
    const auto          primary_dims = wrist_enc.empty() ? static_dims : wrist_dims;
    if (!primary_enc.empty()) {
      assembler_->assemble(*frame, primary_dims.first, primary_dims.second,
                           primary_enc, inputs);
    }
    // If the second image has a different encoding, re-assemble for it by
    // stripping the first image from a copy of the frame.
    if (!wrist_enc.empty() && !static_enc.empty() && wrist_enc != static_enc) {
      AlignedFrame second_frame;
      if (frame->contains("observation.images.static")) {
        second_frame.set("observation.images.static",
                         frame->get<std::vector<uint8_t>>("observation.images.static"));
      }
      if (frame->contains("observation.state")) {
        second_frame.set("observation.state",
                         frame->get<std::vector<double>>("observation.state"));
      }
      assembler_->assemble(second_frame, static_dims.first, static_dims.second,
                           static_enc, inputs);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "assemble: %s", e.what());
    return;
  }

  // Drop-on-overrun: if a previous run is in flight, skip this tick's request.
  {
    std::lock_guard<std::mutex> lk(inference_mu_);
    if (inference_in_flight_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "inference overrun; dropping tick");
      return;
    }
    inference_in_ = std::move(inputs);
    inference_run_pending_ = true;
    inference_in_flight_ = true;
  }
  inference_cv_.notify_one();
}

void InferenceNode::inference_loop() {
  while (!inference_should_exit_.load()) {
    OnnxRunner::FloatMap local_inputs;
    {
      std::unique_lock<std::mutex> lk(inference_mu_);
      inference_cv_.wait(lk, [&] {
        return inference_should_exit_.load() || inference_run_pending_;
      });
      if (inference_should_exit_.load()) break;
      local_inputs = std::move(inference_in_);
      inference_run_pending_ = false;
    }

    std::vector<std::vector<float>> outputs;
    try {
      outputs = runner_->run(local_inputs);
    } catch (const std::exception & e) {
      if (onnx_error_policy_ == "fatal") {
        RCLCPP_ERROR(get_logger(), "onnx run failed: %s — finishing goal", e.what());
        goal_active_.store(false);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "onnx run failed: %s", e.what());
      }
      {
        std::lock_guard<std::mutex> lk(inference_mu_);
        inference_in_flight_ = false;
      }
      continue;
    }

    if (outputs.empty() || outputs[0].size() % 6 != 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "onnx output[0] size %zu not divisible by 6",
                           outputs.empty() ? 0u : outputs[0].size());
      {
        std::lock_guard<std::mutex> lk(inference_mu_);
        inference_in_flight_ = false;
      }
      continue;
    }
    const std::size_t N = outputs[0].size() / 6;
    for (std::size_t i = 0; i < N; ++i) {
      Action a;
      for (std::size_t d = 0; d < 6; ++d) {
        a.values[d] = static_cast<double>(outputs[0][i * 6 + d]);
      }
      chunk_queue_.push(a);
    }
    {
      std::lock_guard<std::mutex> lk(inference_mu_);
      inference_in_flight_ = false;
    }
  }
}

void InferenceNode::publish_feedback() {
  if (!goal_active_.load() || !active_goal_ || !active_goal_->is_active()) return;
  auto fb = std::make_shared<RunPolicy::Feedback>();
  fb->published_actions = published_actions_.load();
  fb->queue_depth       = static_cast<uint32_t>(chunk_queue_.size());
  fb->status            = "executing";
  active_goal_->publish_feedback(fb);
}

}  // namespace pai_onnx_inference
