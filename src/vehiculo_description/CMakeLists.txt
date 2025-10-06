cmake_minimum_required(VERSION 3.8)
project(vehiculo_description)

find_package(ament_cmake REQUIRED)

# Instalar recursos (launch, urdf y meshes) al area de instalación
 
install(DIRECTORY urdf meshes launch worlds config
  DESTINATION share/${PROJECT_NAME})


ament_package()
