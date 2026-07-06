cmake_minimum_required(VERSION 3.16)
project(pai_aic NONE)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
ament_python_install_package(${PROJECT_NAME})
ament_package()