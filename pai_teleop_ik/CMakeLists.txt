cmake_minimum_required(VERSION 3.22)
project(pai_teleop_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

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
  # The launch_testing pytest plugins break collection on pytest>=9; these
  # tests do not use them.
  set(_pytest_env "PYTEST_ADDOPTS=-p no:launch_testing -p no:launch_ros")
  ament_add_pytest_test(test_ik_solver test/test_ik_solver.py
    ENV ${_pytest_env})
  ament_add_pytest_test(test_se3_filter test/test_se3_filter.py
    ENV ${_pytest_env})
endif()

ament_package()
