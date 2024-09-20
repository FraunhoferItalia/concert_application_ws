cmake_minimum_required(VERSION 3.8)
project(rqt_rviz_drilling_widget)

find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()