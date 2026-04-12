cmake_minimum_required(VERSION 3.8)
project(ramses_robot_description)

find_package(ament_cmake REQUIRED)

# Install URDF
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install meshes (STL files from Onshape export)
install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()