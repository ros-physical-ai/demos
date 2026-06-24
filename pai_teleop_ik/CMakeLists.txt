cmake_minimum_required(VERSION 3.22)
project(pai_teleop_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Reusable, frontend-agnostic Python modules (solver, SE3 filter, ROS utils).
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/ik_servo_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ik_solver test/test_ik_solver.py)
  ament_add_pytest_test(test_se3_filter test/test_se3_filter.py)
endif()

ament_package()
