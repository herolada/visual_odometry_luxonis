cmake_minimum_required(VERSION 3.8)
project(visual_odometry)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
    visual_odometry/visual_odometry.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
