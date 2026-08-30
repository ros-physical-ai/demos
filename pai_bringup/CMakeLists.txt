cmake_minimum_required(VERSION 3.10)
project(pai_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the Python package (launch helpers imported by the launch files).
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch mjcf urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
