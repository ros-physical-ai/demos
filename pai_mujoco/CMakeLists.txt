cmake_minimum_required(VERSION 3.22)
project(pai_mujoco)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
    DIRECTORY
        urdf
        config
        rviz
        launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
