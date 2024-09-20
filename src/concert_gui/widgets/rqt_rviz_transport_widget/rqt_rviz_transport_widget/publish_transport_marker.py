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

import rclpy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
import yaml
import os
from ament_index_python import get_package_share_directory
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import String

class TransportPositionMarkers(Node):
    def __init__(self):

        Node.__init__(self, 'transport_marker_node')


        self.menu_handler = MenuHandler()
        
        self.send_goal_client = ActionClient(
            self, 
            NavigateToPose, 
            '/robot/navigate_frame', 
            callback_group=MutuallyExclusiveCallbackGroup()
        )    
        self.server = InteractiveMarkerServer(self, "transport_marker_server")

    # Create the goal message for the navigation
    def create_goal_msg(self, goal_pose):
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = goal_pose[0]
        goal.pose.pose.position.y = goal_pose[1]
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = -0.3799
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = -0.924
        return goal

    # Send the robot at goal position
    def send_position_goal(self, goal_msg):
        if not self.send_goal_client.wait_for_server(10.0):
            self.get_logger().error("Navigation action server not ready.")
            return

        self._send_goal_future = self.send_goal_client.send_goal_async(goal_msg)

    # Menu to call the robot at the corresponding position
    def menu_callback(self, feedback ):
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            goal_pose = []
            goal_pose.append(feedback.pose.position.x)
            goal_pose.append(feedback.pose.position.y)
            goal_msg = self.create_goal_msg(goal_pose)
            self.send_position_goal(goal_msg)
    
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    # Spawn marker on the map
    def load_marker_positions(self):
        menu = InteractiveMarkerControl()
        menu.interaction_mode = InteractiveMarkerControl.MENU
        menu.name = "transport_position_menu"
        menu.always_visible = True
        self.menu_handler.insert(
            "Send Robot to Transport Position", callback=self.menu_callback)

        yaml_config_path = os.path.join(
            get_package_share_directory("concert_main_gui"),
            "config/yaml_files",
            "transport_positions.yaml",
        )

        with open(yaml_config_path, 'r') as yaml_config_file:
            yaml_conf = yaml.safe_load(yaml_config_file)        
            positions = yaml_conf['positions']

            for position in positions:
                int_marker = InteractiveMarker()
                int_marker.header.frame_id = "map"
                int_marker.name = position.get('name')
                int_marker.description = position.get('name')
                int_marker.pose.position.x = position.get('x')
                int_marker.pose.position.y = position.get('y')

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


def main(args=None):
    rclpy.init(args=args)
    position_marker = TransportPositionMarkers()
    position_marker.load_marker_positions()
    rclpy.spin(position_marker)

if __name__=="__main__":
    main()
