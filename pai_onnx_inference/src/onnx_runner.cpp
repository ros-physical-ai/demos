#include "pai_onnx_inference/onnx_runner.hpp"

#include <algorithm>
#include <iostream>
#include <memory>

#include <onnxruntime/core/session/onnxruntime_cxx_api.h>

namespace pai_onnx_inference {

struct OnnxRunner::Impl {
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "pai_onnx_inference"};
  std::unique_ptr<Ort::SessionOptions> opts;
  std::unique_ptr<Ort::Session> session;
  std::unique_ptr<Ort::MemoryInfo> mem_info;
  std::vector<std::vector<int64_t>> input_shapes;
};

OnnxRunner::OnnxRunner() : impl_(std::make_unique<Impl>()) {}
OnnxRunner::~OnnxRunner() = default;

namespace {

std::vector<std::string> available_eps() {
  return Ort::GetAvailableProviders();
}

}  // namespace

void OnnxRunner::load(const std::string & model_path,
                      const std::string & ep_requested,
                      const std::string & ep_policy) {
  impl_->opts = std::make_unique<Ort::SessionOptions>();
  impl_->mem_info = std::make_unique<Ort::MemoryInfo>(
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

  const auto eps = available_eps();
  const bool has_cpu  = std::find(eps.begin(), eps.end(), "CPUExecutionProvider") != eps.end();
  const bool has_cuda = std::find(eps.begin(), eps.end(), "CUDAExecutionProvider") != eps.end();

  std::string ep = ep_requested;
  if (ep == "auto") ep = has_cuda ? "cuda" : "cpu";
  if (ep == "cuda" && !has_cuda) {
    if (ep_policy == "strict") {
      throw std::runtime_error("CUDA EP requested but not available");
    }
    std::cerr << "[WARN] CUDA EP not available; falling back to CPU EP\n";
    ep = "cpu";
  }
  if (ep == "cpu" && !has_cpu) {
    throw std::runtime_error("CPU EP not available — broken ONNX Runtime build");
  }

  if (ep == "cuda") {
    OrtCUDAProviderOptions cuda_opts{};
    cuda_opts.device_id = 0;
    impl_->opts->AppendExecutionProvider_CUDA(cuda_opts);
  }
  impl_->opts->SetIntraOpNumThreads(1);
  impl_->opts->SetInterOpNumThreads(1);

  try {
    impl_->session = std::make_unique<Ort::Session>(
        impl_->env, model_path.c_str(), *impl_->opts);
  } catch (const std::exception & e) {
    throw std::runtime_error("Ort::Session load failed for '" + model_path +
                             "': " + e.what());
  }
  active_ep_ = (ep == "cuda") ? "CUDAExecutionProvider" : "CPUExecutionProvider";

  // Introspect I/O.
  input_names_  = impl_->session->GetInputNames();
  output_names_ = impl_->session->GetOutputNames();
  impl_->input_shapes.clear();
  for (std::size_t i = 0; i < impl_->session->GetInputCount(); ++i) {
    auto ti = impl_->session->GetInputTypeInfo(i);
    impl_->input_shapes.push_back(ti.GetTensorTypeAndShapeInfo().GetShape());
  }
}

std::vector<std::vector<float>> OnnxRunner::run(const FloatMap & inputs) {
  if (!impl_ || !impl_->session) throw std::runtime_error("OnnxRunner::run called before load");

  std::vector<Ort::Value> ort_inputs;
  ort_inputs.reserve(input_names_.size());
  std::vector<const char *> in_names;
  in_names.reserve(input_names_.size());

  for (std::size_t i = 0; i < input_names_.size(); ++i) {
    auto it = inputs.find(input_names_[i]);
    if (it == inputs.end()) {
      throw std::invalid_argument("OnnxRunner::run: missing input '" + input_names_[i] + "'");
    }
    const auto & shape = impl_->input_shapes[i];
    int64_t expected = 1;
    for (auto d : shape) if (d > 0) expected *= d;
    if (static_cast<int64_t>(it->second.size()) != expected) {
      throw std::invalid_argument(
          "OnnxRunner::run: input '" + input_names_[i] +
          "' expected " + std::to_string(expected) + " floats, got " +
          std::to_string(it->second.size()));
    }
    auto tensor = Ort::Value::CreateTensor<float>(
        *impl_->mem_info, const_cast<float *>(it->second.data()),
        it->second.size(), shape.data(), shape.size());
    ort_inputs.push_back(std::move(tensor));
    in_names.push_back(input_names_[i].c_str());
  }

  std::vector<const char *> out_names;
  out_names.reserve(output_names_.size());
  for (const auto & n : output_names_) out_names.push_back(n.c_str());

  auto ort_outputs = impl_->session->Run(Ort::RunOptions{nullptr},
                                         in_names.data(), ort_inputs.data(), ort_inputs.size(),
                                         out_names.data(), out_names.size());

  std::vector<std::vector<float>> result;
  result.reserve(ort_outputs.size());
  for (auto & out : ort_outputs) {
    auto info = out.GetTensorTypeAndShapeInfo();
    auto count = info.GetElementCount();
    auto * data = out.GetTensorData<float>();
    result.emplace_back(data, data + count);
  }
  return result;
}

}  // namespace pai_onnx_inference
