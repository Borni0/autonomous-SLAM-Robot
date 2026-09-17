cmake_minimum_required(VERSION 3.16)
project(rover_description LANGUAGES)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
