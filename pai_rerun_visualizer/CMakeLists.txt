cmake_minimum_required(VERSION 3.8)
project(pai_rerun_visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install the Python package (importable modules).
ament_python_install_package(${PROJECT_NAME})

# Install the node entry point as an executable runnable via `ros2 run`.
install(PROGRAMS
  scripts/visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and configuration.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
