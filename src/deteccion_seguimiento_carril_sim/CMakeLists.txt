cmake_minimum_required(VERSION 3.8)
project(deteccion_seguimiento_carril_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf worlds models rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
