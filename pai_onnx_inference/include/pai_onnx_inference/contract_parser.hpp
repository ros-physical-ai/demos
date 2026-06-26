#pragma once

#include <string>
#include "pai_onnx_inference/contract.hpp"

namespace pai_onnx_inference {

// Parse a Rosetta contract YAML file. Throws std::runtime_error on any error.
// The file format mirrors pai_data_collection/config/rosetta/so_arm101.yaml.
Contract parse_contract(const std::string & path);

}  // namespace pai_onnx_inference