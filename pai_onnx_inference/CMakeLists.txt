cmake_minimum_required(VERSION 3.16)
project(pai_onnx_inference VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(PAI_ONNX_BUILD_TESTS "Build unit and integration tests" ON)
option(PAI_ONNX_USE_CUDA   "Enable ONNX Runtime CUDA EP at compile time" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosetta_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

# onnxruntime shipped by conda-forge's onnxruntime-cpp package. The conda
# package's CMake config exports `onnxruntime::onnxruntime` as an IMPORTED
# target but does not set IMPORTED_LOCATION. We locate the .so + headers
# directly. Header layout:
#   include/onnxruntime/core/session/onnxruntime_cxx_api.h
# The conda-forge `onnxruntime-cpp` package ships:
#   - libonnxruntime.so in $CONDA_PREFIX/lib
#   - C++ wrapper at $CONDA_PREFIX/include/onnxruntime/core/session/onnxruntime_cxx_api.h
# Its CMake config is incomplete (imported target has no IMPORTED_LOCATION),
# so we link by plain library name and add the include path explicitly.
find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES core/session/onnxruntime_cxx_api.h
  HINTS "${CMAKE_PREFIX_PATH}/include/onnxruntime")
if(NOT ONNXRUNTIME_INCLUDE_DIR)
  message(FATAL_ERROR "onnxruntime headers not found under ${CMAKE_PREFIX_PATH}/include/onnxruntime")
endif()
message(STATUS "onnxruntime headers: ${ONNXRUNTIME_INCLUDE_DIR}")

add_library(pai_onnx_inference SHARED
  src/contract_parser.cpp
  src/aligned_buffer.cpp
  src/chunk_queue.cpp
  src/onnx_runner.cpp
  src/observation_assembler.cpp
  src/inference_node.cpp)
target_include_directories(pai_onnx_inference
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
# Use the keyword `target_link_libraries` form throughout (modern CMake;
# `ament_target_dependencies` is deprecated and incompatible).
target_include_directories(pai_onnx_inference SYSTEM PUBLIC ${ONNXRUNTIME_INCLUDE_DIR})
target_link_libraries(pai_onnx_inference
  PUBLIC
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    sensor_msgs::sensor_msgs__rosidl_generator_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    std_msgs::std_msgs__rosidl_generator_cpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
    rosetta_interfaces::rosetta_interfaces__rosidl_generator_cpp
    rosetta_interfaces::rosetta_interfaces__rosidl_typesupport_cpp
    yaml-cpp::yaml-cpp
    onnxruntime)

if(PAI_ONNX_USE_CUDA)
  target_compile_definitions(pai_onnx_inference PUBLIC PAI_ONNX_ENABLE_CUDA=1)
endif()

add_executable(pai_onnx_inference_node src/main.cpp)
target_link_libraries(pai_onnx_inference_node PRIVATE pai_onnx_inference)

install(TARGETS pai_onnx_inference pai_onnx_inference_node
  EXPORT export_pai_onnx_inference
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING AND PAI_ONNX_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

ament_export_targets(export_pai_onnx_inference HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action sensor_msgs std_msgs rosetta_interfaces yaml-cpp)

if(PAI_ONNX_BUILD_TESTS)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()