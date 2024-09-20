cmake_minimum_required(VERSION 3.8)
project(concert_mission_generator)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behavior_tree_msgs REQUIRED)

install(DIRECTORY config scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/send_behavior_tree_client.py
  scripts/interactive_marker_base.py
  scripts/drilling_mission_generator.py
  scripts/IIT_drilling_mission_generator.py
  scripts/transport_mission_generator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()