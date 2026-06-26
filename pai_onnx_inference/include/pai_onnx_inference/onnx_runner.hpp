#pragma once

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace Ort {
struct Env;
struct Session;
struct SessionOptions;
struct MemoryInfo;
}  // namespace Ort

namespace pai_onnx_inference {

// OnnxRunner wraps an `Ort::Session` and provides:
//  - EP selection at load time (cpu, cuda, or auto),
//  - strict / fallback policy on EP availability,
//  - introspection of I/O names,
//  - a single-threaded `run()` that takes a name->float-vector map.
//
// Thread-safety: NOT thread-safe for `run()`. InferenceNode calls `run`
// on a dedicated thread.
class OnnxRunner {
 public:
  OnnxRunner();
  ~OnnxRunner();

  OnnxRunner(const OnnxRunner &) = delete;
  OnnxRunner & operator=(const OnnxRunner &) = delete;

  // ep_requested: "cpu" | "cuda" | "auto"
  // ep_policy:    "strict" (throw) | "fallback" (downgrade to CPU, log warning)
  void load(const std::string & model_path,
            const std::string & ep_requested,
            const std::string & ep_policy);

  const std::vector<std::string> & input_names()  const { return input_names_;  }
  const std::vector<std::string> & output_names() const { return output_names_; }
  const std::string & active_ep() const { return active_ep_; }

  // Run synchronously. Throws std::invalid_argument if a required input is
  // missing, std::runtime_error if the underlying ORT call fails.
  using FloatMap = std::map<std::string, std::vector<float>>;
  std::vector<std::vector<float>> run(const FloatMap & inputs);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::string active_ep_;
};

}  // namespace pai_onnx_inference
