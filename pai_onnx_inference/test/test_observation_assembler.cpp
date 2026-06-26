#include <gtest/gtest.h>
#include <cstdint>
#include <string>
#include <vector>
#include "pai_onnx_inference/aligned_buffer.hpp"
#include "pai_onnx_inference/contract.hpp"
#include "pai_onnx_inference/observation_assembler.hpp"
#include "pai_onnx_inference/onnx_runner.hpp"

using pai_onnx_inference::AlignedFrame;
using pai_onnx_inference::Contract;
using pai_onnx_inference::ObservationAssembler;
using pai_onnx_inference::ObservationSpec;
using pai_onnx_inference::OnnxRunner;
using pai_onnx_inference::Payload;
using pai_onnx_inference::QoS;

static Contract make_test_contract() {
  Contract c;
  c.fps = 50;
  {
    ObservationSpec s;
    s.key = "observation.images.wrist";
    s.topic = "/wrist_camera/image_raw";
    s.ros_type = "sensor_msgs/msg/Image";
    s.image_resize = {4, 4};
    s.align_strategy = "hold";
    s.align_tol_ms = 100;
    s.qos = QoS{"best_effort", "keep_last", 10};
    c.observations.push_back(s);
  }
  {
    ObservationSpec s;
    s.key = "observation.state";
    s.topic = "/joint_states";
    s.ros_type = "sensor_msgs/msg/JointState";
    s.selector_names = {"a", "b", "c", "d", "e", "f"};
    s.align_strategy = "hold";
    s.align_tol_ms = 100;
    s.qos = QoS{"reliable", "keep_last", 50};
    s.unit_conversion = "rad2deg";
    c.observations.push_back(s);
  }
  return c;
}

TEST(ObservationAssembler, ImageResizedAndLaidOutNCHW) {
  auto contract = make_test_contract();
  ObservationAssembler asmb(contract);

  // 2x2 BGR image: pixels (B,G,R)
  //   row0: (1,2,3) (4,5,6)
  //   row1: (7,8,9) (10,11,12)
  std::vector<uint8_t> img = {1,2,3, 4,5,6,  7,8,9, 10,11,12};

  AlignedFrame frame;
  frame.set("observation.images.wrist", Payload{img});
  frame.set("observation.state", Payload{std::vector<double>{0.1, -0.2, 0.3, -0.4, 0.5, -0.6}});

  OnnxRunner::FloatMap out;
  asmb.assemble(frame, /*src_h=*/2, /*src_w=*/2, /*src_encoding=*/"bgr8", out);

  // Image: NCHW float32, 1*3*4*4 = 48 floats in [0, 1].
  ASSERT_TRUE(out.count("observation.images.wrist"));
  EXPECT_EQ(out["observation.images.wrist"].size(), 3u * 4u * 4u);
  for (auto v : out["observation.images.wrist"]) {
    EXPECT_GE(v, 0.0f);
    EXPECT_LE(v, 1.0f);
  }
  // After BGR→RGB swap, channel 0 (R) at pixel (0,0) should be 3/255 = 0.0117…
  // The output layout is NCHW with shape (1, 3, 4, 4). After resize from 2x2
  // to 4x4 (nearest neighbor), every destination pixel maps back to its 2x2
  // source — so all four corners get source (0,0) or (1,1) etc. Specifically
  // pixel (0,0) of the resized output corresponds to source (0,0) which had
  // BGR=(1,2,3); after swap that's RGB=(3,2,1). The first channel of NCHW is R
  // = 3/255.
  const float R00 = out["observation.images.wrist"][0 * 4 * 4 + 0 * 4 + 0];
  EXPECT_NEAR(R00, 3.0f / 255.0f, 1e-4f);
}

TEST(ObservationAssembler, StateConvertedRadToDeg) {
  auto contract = make_test_contract();
  ObservationAssembler asmb(contract);

  AlignedFrame frame;
  frame.set("observation.state", Payload{std::vector<double>{0.0, 0.1, -0.1, 0.5, -0.5, 1.0}});

  OnnxRunner::FloatMap out;
  asmb.assemble(frame, 2, 2, "bgr8", out);

  ASSERT_TRUE(out.count("observation.state"));
  ASSERT_EQ(out["observation.state"].size(), 6u);
  const double kRadToDeg = 57.29577951308232;
  EXPECT_NEAR(out["observation.state"][0], 0.0  * kRadToDeg, 1e-3);
  EXPECT_NEAR(out["observation.state"][1], 0.1  * kRadToDeg, 1e-3);
  EXPECT_NEAR(out["observation.state"][2], -0.1 * kRadToDeg, 1e-3);
  EXPECT_NEAR(out["observation.state"][5], 1.0  * kRadToDeg, 1e-3);
}

TEST(ObservationAssembler, RgbEncodingSkipsSwap) {
  auto contract = make_test_contract();
  ObservationAssembler asmb(contract);

  // RGB-encoded image (no swap).
  std::vector<uint8_t> img = {1,2,3, 4,5,6,  7,8,9, 10,11,12};
  AlignedFrame frame;
  frame.set("observation.images.wrist", Payload{img});
  frame.set("observation.state", Payload{std::vector<double>{0,0,0,0,0,0}});

  OnnxRunner::FloatMap out;
  asmb.assemble(frame, 2, 2, "rgb8", out);

  // No swap: channel 0 of NCHW (R) at (0,0) should be 1/255.
  const float R00 = out["observation.images.wrist"][0];
  EXPECT_NEAR(R00, 1.0f / 255.0f, 1e-4f);
}

TEST(ObservationAssembler, MissingFrameKeyIsSkipped) {
  auto contract = make_test_contract();
  ObservationAssembler asmb(contract);

  // Only push the image; state is missing.
  std::vector<uint8_t> img(2 * 2 * 3, 0);
  AlignedFrame frame;
  frame.set("observation.images.wrist", Payload{img});

  OnnxRunner::FloatMap out;
  asmb.assemble(frame, 2, 2, "bgr8", out);

  EXPECT_TRUE(out.count("observation.images.wrist"));
  EXPECT_FALSE(out.count("observation.state"));
}