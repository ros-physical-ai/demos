find_package(ament_cmake_gtest REQUIRED)
find_package(Threads REQUIRED)

set(_fixture "${CMAKE_CURRENT_SOURCE_DIR}/fixtures/identity_policy.onnx")
if(NOT EXISTS "${_fixture}")
  message(FATAL_ERROR "Fixture ONNX missing — run fixtures/generate.py")
endif()

add_definitions(-DPAI_ONNX_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")

ament_add_gtest(test_contract_parser     test_contract_parser.cpp)
ament_add_gtest(test_aligned_buffer      test_aligned_buffer.cpp)
ament_add_gtest(test_chunk_queue         test_chunk_queue.cpp)
ament_add_gtest(test_onnx_runner         test_onnx_runner.cpp)
ament_add_gtest(test_observation_assembler test_observation_assembler.cpp)
ament_add_gtest(test_inference_node      test_inference_node.cpp)
# Plain form for compatibility with ament_add_gtest internals.
target_link_libraries(test_contract_parser       pai_onnx_inference)
target_link_libraries(test_aligned_buffer        pai_onnx_inference)
target_link_libraries(test_chunk_queue           pai_onnx_inference Threads::Threads)
target_link_libraries(test_onnx_runner           pai_onnx_inference)
target_link_libraries(test_observation_assembler pai_onnx_inference)
target_link_libraries(test_inference_node        pai_onnx_inference rclcpp::rclcpp)