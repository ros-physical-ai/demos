#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "pai_onnx_inference/aligned_buffer.hpp"
#include "pai_onnx_inference/contract.hpp"
#include "pai_onnx_inference/onnx_runner.hpp"

namespace pai_onnx_inference {

// Build the ONNX input tensor map from a single aligned observation frame.
//
// Image preprocessing: BGR/RGB auto from `src_encoding`, resize to
// (image_resize.first, image_resize.second) using nearest-neighbor,
// transpose HWC->CHW, scale to [0,1] float32.
//
// State vector: gather entries from the JointState-shaped payload by name
// (selector_names), convert rad->deg if `unit_conversion` == "rad2deg".
class ObservationAssembler {
 public:
  explicit ObservationAssembler(Contract contract);

  void assemble(const AlignedFrame & frame,
                std::int32_t src_h,
                std::int32_t src_w,
                const std::string & src_encoding,
                OnnxRunner::FloatMap & out) const;

 private:
  Contract contract_;
};

}  // namespace pai_onnx_inference
