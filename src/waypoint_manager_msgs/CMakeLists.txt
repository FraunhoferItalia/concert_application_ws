cmake_minimum_required(VERSION 3.5)
project(waypoint_manager_msgs)

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
find_package(moveit_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(actionlib_msgs REQUIRED)

set(msg_files
  "msg/Action.msg"
  "msg/Motion.msg"
  "msg/CartesianTarget.msg"
  "msg/JointTarget.msg"
  "msg/Target.msg"
)

set(srv_files
  "srv/GetAction.srv"
  "srv/GetActionsList.srv"
  "srv/MotionTrigger.srv"
  "srv/WaypointWrite.srv"
  "srv/WaypointLoad.srv"
)

set(action_files
  "action/General.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs moveit_msgs builtin_interfaces actionlib_msgs
  ADD_LINTER_TESTS
)

install(FILES 
  utils/wrappers.py
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()