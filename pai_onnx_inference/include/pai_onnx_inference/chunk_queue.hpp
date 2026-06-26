#pragma once

#include <array>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <optional>
#include <vector>

namespace pai_onnx_inference {

struct Action {
  std::array<double, 6> values{};  // joint targets in DEGREES (pre-conversion)
};

class ChunkQueue {
 public:
  void push(const Action & a) {
    {
      std::lock_guard<std::mutex> lk(mu_);
      queue_.clear();   // a new chunk replaces any leftover chunk
      queue_.push_back(a);
    }
    cv_.notify_one();
  }

  std::optional<Action> pop_front() {
    std::lock_guard<std::mutex> lk(mu_);
    if (queue_.empty()) return std::nullopt;
    Action a = queue_.front();
    queue_.erase(queue_.begin());
    return a;
  }

  std::size_t size() const {
    std::lock_guard<std::mutex> lk(mu_);
    return queue_.size();
  }

  void clear() {
    std::lock_guard<std::mutex> lk(mu_);
    queue_.clear();
  }

 private:
  mutable std::mutex mu_;
  std::condition_variable cv_;
  std::vector<Action> queue_;
};

}  // namespace pai_onnx_inference
