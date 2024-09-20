cmake_minimum_required(VERSION 3.5)
project(concert_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(srv_files
  "srv/String.srv"
  "srv/MoveMarker.srv"
  "srv/MarkerService.srv"
  "msg/SelectedMarkers.msg"
)

set(action_files
  "action/General.action"
  "action/MoveUntilContact.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

install(
  FILES mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()