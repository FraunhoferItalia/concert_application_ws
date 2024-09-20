#!/usr/bin/env python3
# Copyright 2022-2024 Fraunhofer Italia Research

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Copyright 2022 Fraunhofer Italia Research. All Rights Reserved.
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from send_behavior_tree_client import LoadBehaviorTreeTester
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation as R
import rclpy

# import rospkg
from geometry_msgs.msg import Pose


from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from visualization_msgs.msg import Marker

from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA

import yaml


class ConfigurationException(BaseException):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)


class TransportMissionGenerator(LoadBehaviorTreeTester):
    def __init__(self, name: str):
        super().__init__(name)
        self.menu_handler = MenuHandler()
        self.server = InteractiveMarkerServer(self, "transport_marker_server")

    def create_goal_msg(self, goal_pose):
        goal = Pose()

        goal.position.x = goal_pose[0]
        goal.position.y = goal_pose[1]
        goal.position.z = 0.0
        goal.orientation.w = -0.3799
        goal.orientation.x = 0.0
        goal.orientation.y = 0.0
        goal.orientation.z = -0.924
        r = R.from_quat(
            [
                goal.orientation.x,
                goal.orientation.y,
                goal.orientation.z,
                goal.orientation.w,
            ]
        )
        rpy = r.as_euler("zxy", degrees=False)
        theta = rpy[0]
        return goal.position.x, goal.position.y, theta

    def menu_callback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            goal_pose = []
            goal_pose.append(feedback.pose.position.x)
            goal_pose.append(feedback.pose.position.y)
            goal_msg = self.create_goal_msg(goal_pose)
            self.dispatch_mission_clbk(goal_msg)

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def load_marker_positions(self):
        menu = InteractiveMarkerControl()
        menu.interaction_mode = InteractiveMarkerControl.MENU
        menu.name = "transport_position_menu"
        menu.always_visible = True
        self.menu_handler.insert(
            "Send Robot to Transport Position", callback=self.menu_callback
        )

        yaml_config_path = os.path.join(
            get_package_share_directory("concert_main_gui"),
            "config/yaml_files",
            "transport_positions.yaml",
        )

        with open(yaml_config_path, "r") as yaml_config_file:
            yaml_conf = yaml.safe_load(yaml_config_file)
            positions = yaml_conf["positions"]

            for position in positions:
                int_marker = InteractiveMarker()
                int_marker.header.frame_id = "map"
                int_marker.name = position.get("name")
                int_marker.description = position.get("name")
                int_marker.pose.position.x = position.get("x")
                int_marker.pose.position.y = position.get("y")

                scale = Vector3()
                scale.x = 0.8
                scale.y = 0.8
                scale.z = 0.1

                color = ColorRGBA()
                color.r = 0.09
                color.g = 0.61
                color.b = 0.49
                color.a = 1.0

                box_marker = Marker(type=Marker.CUBE, scale=scale, color=color)
                menu.markers.append(box_marker)

                int_marker.controls.append(menu)

                self.server.insert(int_marker)
                self.server.applyChanges()
                self.menu_handler.apply(self.server, int_marker.name)

    def dispatch_mission_clbk(self, position):
        self.get_logger().info(
            f"\nMobile base pose: \nx:{position[0]}\ny:{position[1]}\ntheta:{position[2]}\n"
        )

        input_ports = {
            "mobile_base_pose": f"{position[0]};{position[1]};{position[2]}",
        }

        self.load_and_send(
            "/concert_application_ws/src/concert_mission_generator/trees/transport.xml",
            input_ports,
        )
        return


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = TransportMissionGenerator("transport_mission_generator")
    try:
        node.load_marker_positions()
    except ConfigurationException:
        rclpy.shutdown()
        return

    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.destroy_node()
        node.get_logger().info("Keyboard interrupt, shutting down.\n")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
