cmake_minimum_required(VERSION 3.22)
project(pai_rviz_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Reusable, RViz-agnostic Python modules (e.g. the differential IK solver).
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/interactive_ik_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
