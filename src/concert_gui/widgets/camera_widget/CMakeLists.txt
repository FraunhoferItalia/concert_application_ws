cmake_minimum_required(VERSION 3.8)
project(camera_widget)

find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

install(FILES
  DESTINATION share/${PROJECT_NAME}
)

ament_package()