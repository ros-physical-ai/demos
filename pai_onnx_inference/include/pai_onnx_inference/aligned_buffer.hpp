#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <variant>
#include <vector>

namespace pai_onnx_inference {

using Payload = std::variant<int, double, std::vector<double>, std::vector<uint8_t>, std::string>;

class AlignedFrame {
 public:
  void set(std::string key, Payload value) { values_[std::move(key)] = std::move(value); }

  template <typename T>
  T get(std::string_view key) const {
    auto it = values_.find(std::string{key});
    if (it == values_.end()) {
      throw std::out_of_range("AlignedFrame: missing key '" + std::string{key} + "'");
    }
    if (auto p = std::get_if<T>(&it->second)) return *p;
    throw std::bad_variant_access();
  }

  bool contains(std::string_view key) const {
    return values_.find(std::string{key}) != values_.end();
  }

 private:
  std::unordered_map<std::string, Payload> values_;
};

// Per-key last-value buffer with a hold-on-stale strategy.
//
// Thread-safety: NOT thread-safe. All calls happen on the ROS executor.
class AlignedBuffer {
 public:
  explicit AlignedBuffer(std::int32_t tol_ms) : tol_ns_(tol_ms * 1'000'000) {}

  // Push a new observation; replaces the stored value for this key.
  void push(std::string key, std::int64_t stamp_ns, Payload payload) {
    slots_[std::move(key)] = Slot{stamp_ns, std::move(payload)};
  }

  // Pop a frame at sim time `now_ns`. A key is included if it has been pushed
  // and (now_ns - stamp_ns) <= tol_ns_; otherwise the key is omitted.
  // Returns std::nullopt if NO keys are still fresh at this `now_ns`.
  std::optional<AlignedFrame> pop_at(std::int64_t now_ns) {
    AlignedFrame f;
    bool any = false;
    for (auto it = slots_.begin(); it != slots_.end();) {
      const auto age = now_ns - it->second.stamp_ns;
      if (age < 0) { ++it; continue; }
      if (age > tol_ns_) {
        it = slots_.erase(it);
        continue;
      }
      f.set(it->first, it->second.payload);
      any = true;
      ++it;
    }
    if (!any) return std::nullopt;
    return f;
  }

  void clear() { slots_.clear(); }

 private:
  struct Slot { std::int64_t stamp_ns; Payload payload; };
  std::unordered_map<std::string, Slot> slots_;
  std::int64_t tol_ns_;
};

}  // namespace pai_onnx_inference
