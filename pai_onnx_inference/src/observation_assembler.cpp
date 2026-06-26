#include "pai_onnx_inference/observation_assembler.hpp"

#include <stdexcept>

namespace pai_onnx_inference {

ObservationAssembler::ObservationAssembler(Contract contract)
    : contract_(std::move(contract)) {}

namespace {

// Nearest-neighbor resize from (src_h, src_w) to (dst_h, dst_w).
// Returns float32 CHW [0,1] (already scaled, ready for ONNX).
std::vector<float> resize_hwc_to_chw_f32(const uint8_t * src,
                                         std::int32_t src_h, std::int32_t src_w,
                                         std::int32_t dst_h, std::int32_t dst_w) {
  std::vector<float> out(static_cast<std::size_t>(3) * dst_h * dst_w);
  for (int c = 0; c < 3; ++c) {
    for (int y = 0; y < dst_h; ++y) {
      int sy = y * src_h / dst_h;
      for (int x = 0; x < dst_w; ++x) {
        int sx = x * src_w / dst_w;
        out[(c * dst_h + y) * dst_w + x] =
            static_cast<float>(src[(sy * src_w + sx) * 3 + c]) / 255.0f;
      }
    }
  }
  return out;
}

}  // namespace

void ObservationAssembler::assemble(const AlignedFrame & frame,
                                    std::int32_t src_h,
                                    std::int32_t src_w,
                                    const std::string & src_encoding,
                                    OnnxRunner::FloatMap & out) const {
  for (const auto & obs : contract_.observations) {
    if (!frame.contains(obs.key)) continue;

    if (obs.key.rfind("observation.images.", 0) == 0) {
      const auto & img_bytes = frame.get<std::vector<uint8_t>>(obs.key);
      const int64_t expected = static_cast<int64_t>(src_h) * src_w * 3;
      if (static_cast<int64_t>(img_bytes.size()) != expected) {
        throw std::runtime_error("assemble: image size " +
            std::to_string(img_bytes.size()) + " != expected " +
            std::to_string(expected));
      }
      if (obs.image_resize.first <= 0 || obs.image_resize.second <= 0) {
        throw std::runtime_error("assemble: missing image.resize for " + obs.key);
      }
      const bool swap_rb = (src_encoding == "bgr8");
      std::vector<float> chw;
      if (swap_rb) {
        std::vector<uint8_t> rgb(img_bytes.size());
        for (int i = 0; i < src_h * src_w; ++i) {
          rgb[i*3 + 0] = img_bytes[i*3 + 2];
          rgb[i*3 + 1] = img_bytes[i*3 + 1];
          rgb[i*3 + 2] = img_bytes[i*3 + 0];
        }
        chw = resize_hwc_to_chw_f32(rgb.data(), src_h, src_w,
                                    obs.image_resize.first, obs.image_resize.second);
      } else {
        chw = resize_hwc_to_chw_f32(img_bytes.data(), src_h, src_w,
                                    obs.image_resize.first, obs.image_resize.second);
      }
      out[obs.key] = std::move(chw);
    } else if (obs.key == "observation.state") {
      const auto & values = frame.get<std::vector<double>>(obs.key);
      if (static_cast<int64_t>(values.size()) !=
          static_cast<int64_t>(obs.selector_names.size())) {
        throw std::runtime_error("assemble: state size " +
            std::to_string(values.size()) + " != selector size " +
            std::to_string(obs.selector_names.size()));
      }
      const double kRadToDeg = 57.29577951308232;
      const bool do_conv = (obs.unit_conversion == "rad2deg");
      std::vector<float> f32(values.size());
      for (std::size_t i = 0; i < values.size(); ++i) {
        const double v = do_conv ? values[i] * kRadToDeg : values[i];
        f32[i] = static_cast<float>(v);
      }
      out[obs.key] = std::move(f32);
    }
  }
}

}  // namespace pai_onnx_inference
