#include <gtest/gtest.h>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>
#include "pai_onnx_inference/onnx_runner.hpp"

namespace fs = std::filesystem;
using pai_onnx_inference::OnnxRunner;

static std::string fixture_path() {
  return (fs::path(PAI_ONNX_FIXTURE_DIR) / "identity_policy.onnx").string();
}

TEST(OnnxRunner, LoadsFixtureOnCpu) {
  OnnxRunner runner;
  ASSERT_NO_THROW(runner.load(fixture_path(), /*ep_requested=*/"cpu", /*ep_policy=*/"strict"));
  EXPECT_EQ(runner.active_ep(), "CPUExecutionProvider");
  EXPECT_EQ(runner.input_names(), (std::vector<std::string>{"observation.state"}));
  EXPECT_EQ(runner.output_names(), (std::vector<std::string>{"action"}));
}

TEST(OnnxRunner, RunProducesCorrectShape) {
  OnnxRunner runner;
  runner.load(fixture_path(), "cpu", "strict");
  OnnxRunner::FloatMap inputs;
  inputs["observation.state"] = std::vector<float>(6, 0.0f);
  auto outs = runner.run(inputs);
  ASSERT_EQ(outs.size(), 1u);
  EXPECT_EQ(outs[0].size(), 6u);
  for (auto v : outs[0]) EXPECT_FLOAT_EQ(v, 0.0f);
}

TEST(OnnxRunner, MissingInputThrows) {
  OnnxRunner runner;
  runner.load(fixture_path(), "cpu", "strict");
  EXPECT_THROW(runner.run({}), std::invalid_argument);
}

TEST(OnnxRunner, StrictPolicyRefusesIfCudaUnavailable) {
#ifdef PAI_ONNX_ENABLE_CUDA
  GTEST_SKIP() << "CUDA EP compiled in; cannot test the strict-refusal path.";
#else
  OnnxRunner runner;
  EXPECT_THROW(runner.load(fixture_path(), "cuda", "strict"),
               std::runtime_error);
#endif
}

TEST(OnnxRunner, FallbackPolicyDowngradesToCpu) {
#ifdef PAI_ONNX_ENABLE_CUDA
  GTEST_SKIP() << "CUDA EP compiled in; fallback path does not apply.";
#else
  OnnxRunner runner;
  ASSERT_NO_THROW(runner.load(fixture_path(), "cuda", "fallback"));
  EXPECT_EQ(runner.active_ep(), "CPUExecutionProvider");
#endif
}