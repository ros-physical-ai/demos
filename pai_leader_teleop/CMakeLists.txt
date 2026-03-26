cmake_minimum_required(VERSION 3.10)
project(pai_leader_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/leader_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
