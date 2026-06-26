#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "pai_onnx_inference/aligned_buffer.hpp"
#include "pai_onnx_inference/chunk_queue.hpp"
#include "pai_onnx_inference/contract.hpp"
#include "pai_onnx_inference/observation_assembler.hpp"
#include "pai_onnx_inference/onnx_runner.hpp"
#include "rosetta_interfaces/action/run_policy.hpp"

namespace pai_onnx_inference {

class InferenceNode : public rclcpp::Node {
 public:
  using RunPolicy = rosetta_interfaces::action::RunPolicy;
  using GoalHandle = rclcpp_action::ServerGoalHandle<RunPolicy>;

  explicit InferenceNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());
  ~InferenceNode() override;

 private:
  // Subscriptions
  void on_image_wrist(const sensor_msgs::msg::Image::SharedPtr msg);
  void on_image_static(const sensor_msgs::msg::Image::SharedPtr msg);
  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &,
                                          std::shared_ptr<const RunPolicy::Goal>);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>);
  void handle_accepted(const std::shared_ptr<GoalHandle>);

  // 50 Hz tick (ROS time)
  void on_tick();

  // Inference thread loop
  void inference_loop();

  // Feedback timer (every 500 ms)
  void publish_feedback();

  // Params
  std::string contract_path_;
  std::string model_path_;
  std::string ep_requested_;
  std::string ep_policy_;
  double observation_timeout_s_{1.0};
  std::int32_t drop_warn_threshold_{5};
  std::string onnx_error_policy_;
  double sim_time_watchdog_s_{2.0};

  // State
  Contract contract_;
  std::unique_ptr<ObservationAssembler> assembler_;
  std::unique_ptr<OnnxRunner> runner_;
  AlignedBuffer aligned_;
  ChunkQueue chunk_queue_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      sub_wrist_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      sub_static_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_states_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;

  rclcpp_action::Server<RunPolicy>::SharedPtr action_server_;
  std::shared_ptr<GoalHandle> active_goal_;
  std::atomic<bool> goal_active_{false};
  std::atomic<uint32_t> published_actions_{0};

  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  // Inference thread synchronization (drop-on-overrun).
  std::thread inference_thread_;
  std::mutex inference_mu_;
  std::condition_variable inference_cv_;
  std::atomic<bool> inference_should_exit_{false};
  bool inference_run_pending_{false};   // guarded by inference_mu_
  OnnxRunner::FloatMap inference_in_;   // guarded by inference_mu_
  bool inference_in_flight_{false};     // guarded by inference_mu_

  // Per-topic image encoding cache (autodetected from incoming Image::encoding).
  std::unordered_map<std::string, std::string> last_image_encoding_;  // key → encoding
  std::unordered_map<std::string, std::pair<int32_t, int32_t>> last_image_dims_;  // key → (h, w)
  std::mutex encoding_mu_;

  // Sim-time watchdog.
  rclcpp::Time last_sim_time_seen_;
};

}  // namespace pai_onnx_inference
