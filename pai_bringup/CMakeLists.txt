cmake_minimum_required(VERSION 3.10)
project(pai_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch scripts urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts and wrappers
install(
  PROGRAMS
    scripts/lerobot_inference_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
