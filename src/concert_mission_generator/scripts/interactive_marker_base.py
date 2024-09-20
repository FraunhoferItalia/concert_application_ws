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

# MissionBase class is extended by painting/drilling tasks...
# InteractiveMarkerBase manages the IM server and the relative
# InteractiveMarkerScalable extends InteractiveMarkerBase adding the scaling feature
# markers/action to update them
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from typing import List

# from interactive_markers.interactive_marker_server import *
# from interactive_markers.menu_handler import *


# ros2
import rclpy

import sys

from visualization_msgs.msg import *
from interactive_markers import InteractiveMarkerServer
from interactive_markers import MenuHandler


tf_topic_frame_id = "drilling_task_frame"
node = None


class InteractiveMarkerBase:
    def __init__(self, node=None):
        self.node = node
        print("Initialazing the new instance of InteractiveMarkerBase")
        self.interactive_marker_server = InteractiveMarkerServer(
            self.node, "rosbim_interactive_marker_server"
        )
        self.interactive_marker_list = []
        self.marker_list = []
        self.scalable_interactive_marker_list = []
        self.actual_marker_pose = Pose()
        self.actual_marker_pose.position.x = 0.0
        self.actual_marker_pose.position.y = 0.0
        self.actual_marker_pose.position.z = 0.0
        self.actual_marker_pose.orientation.x = 0.0
        self.actual_marker_pose.orientation.y = 0.0
        self.actual_marker_pose.orientation.z = 0.0
        self.actual_marker_pose.orientation.w = 1.0

    def clear_interactive_marker_server(self):
        """
        removes all the server's markers and applay changes
        """
        self.interactive_marker_server.clear()
        self.interactive_marker_list = []
        self.marker_list = []
        self.scalable_interactive_marker_list = []
        self.actual_marker_pose = Pose()
        self.actual_marker_pose.position.x = 0.0
        self.actual_marker_pose.position.y = 0.0
        self.actual_marker_pose.position.z = 0.0
        self.actual_marker_pose.orientation.x = 0.0
        self.actual_marker_pose.orientation.y = 0.0
        self.actual_marker_pose.orientation.z = 0.0
        self.actual_marker_pose.orientation.w = 1.0
        self.interactive_marker_server.applyChanges()
        print("All the markers removed")

    def add_new_interactive_marker(
        self,
        name: str,
        frame_id: str = "mission_task_frame",
        description: str = "",
        position: Pose = Pose(),
    ):
        """_summary_

        Args:
            name (str marker&#39;s name): _description_
            frame_id (str to identify the id, optional): _description_. Defaults to "mission_task_frame".
            description (str, optional): _description_. Defaults to "".
            position (Pose, optional): _description_. Defaults to Pose().
        """
        int_marker = InteractiveMarker()
        int_marker.name = name
        int_marker.header.frame_id = frame_id
        int_marker.description = description
        int_marker.pose = position

        print(f"adding new interactive marker in position {position}")

        self.interactive_marker_list.append(int_marker)

        self.interactive_marker_server.applyChanges()

    def add_new_marker(
        self,
        marker_type: Marker.type = Marker.CUBE,
        marker_scale: [float] = [0.3, 0.3, 0.3],
        marker_color: [float] = [1.0, 0.0, 0.5, 1.0],
        position: [float] = [0.0, 0.0, 0.0],
        orientation: [float] = [0.0, 0.0, 0.0],
    ):
        """_summary_

        Args:
            marker_type (Marker.ARROW, CUBE, SPHERE, CYLINDER..., optional): _description_. Defaults to Marker.CUBE.
            marker_scale (list of float in form of [x, y, z], optional): _description_. Defaults to [0.3, 0.3, 0.3].
            marker_color (list of float in form of [r, g, b, a], optional): _description_. Defaults to [1.0, 0.0, 0.5, 1.0].
            orientation (list, optional): _description_. Defaults to [ 0, 0, 0].
        """
        marker = Marker()
        marker.type = marker_type
        marker.scale.x = marker_scale[0]
        marker.scale.y = marker_scale[1]
        marker.scale.z = marker_scale[2]

        marker.color.r = marker_color[0]
        marker.color.g = marker_color[1]
        marker.color.b = marker_color[2]
        marker.color.a = marker_color[3]

        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])

        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = float(orientation[0])
        marker.pose.orientation.y = float(orientation[2])
        marker.pose.orientation.z = float(orientation[1])

        self.marker_list.append(marker)

        self.interactive_marker_server.applyChanges()

    def add_control_to_interactive_marker(
        self,
        interactive_marker=None,
        mode: int = 0,
        orientation: List[int] = [
            0.0,
            0.0,
            1.0,
        ],
        marker=None,
    ):
        """_summary_

        Args:
            interactive_marker (_type_, optional): _description_. Defaults to None.
            mode (0 TRANSLATION, 1 ROTATION, 2 SCALE, optional): _description_. Defaults to 0.
            orientation (express in a list wich orientation you want to add the controll specifing the axis, optional): _description_. Defaults to [ 0, 0, 1, ].
            marker (_type_, optional): _description_. Ad a fixed marker to the interactive marker. Defaults to None.
        """
        if interactive_marker == None:
            interactive_marker = self.interactive_marker_list[0]

        control = InteractiveMarkerControl()
        control.always_visible = True

        if marker != None:
            control.markers.append(marker)
        # control.markers.append( interactive_marker_base.marker_list[1])

        control.orientation.x = float(orientation[0])
        control.orientation.y = float(
            orientation[2]
        )  # TODO: TO UNDERSTAND WHY x and y are switched
        control.orientation.z = float(orientation[1])
        control.orientation.w = 1.0

        # control.orientation_mode = InteractiveMarkerControl.FIXED
        control.orientation_mode = InteractiveMarkerControl.INHERIT

        if mode == 0:  # TRANSLATION
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        elif mode == 1:  # ROTATION
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        elif mode == 2:  # SCALE TEST
            self.scalable_interactive_marker_list.append(
                ScalableInteractiveMarker(
                    self.interactive_marker_list[0], self.interactive_marker_server
                )
            )
            self.scalable_interactive_marker_list[
                0
            ].add_scale_control_to_interactive_marker(
                [1, 0, 0]
            )  # scale x

        interactive_marker.controls.append(control)
        self.interactive_marker_server.insert(
            marker=interactive_marker, feedback_callback=self.process_feedback
        )

        self.actual_marker_pose = interactive_marker.pose

        self.interactive_marker_server.applyChanges()

    def add_menu_to_interactive_marker(
        self, menu_name, marker, interactive_marker, dispatch_function
    ):
        menu = InteractiveMarkerControl()

        menu.interaction_mode = InteractiveMarkerControl.MENU
        menu.name = menu_name
        menu.always_visible = True

        self.menu_handler = MenuHandler()

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True

        control.markers.append(marker)

        interactive_marker.controls.append(control)

        self.interactive_marker_server.insert(
            marker=interactive_marker, feedback_callback=self.process_feedback
        )

        self.initMenu(dispatch_function)

        self.menu_handler.apply(self.interactive_marker_server, interactive_marker.name)

        self.interactive_marker_server.applyChanges()

    def add_fixed_marker_to_interactive_marker(
        self, interactive_marker=None, marker=None, orientation=[0, 0, 1]
    ):
        if interactive_marker == None:
            interactive_marker = self.interactive_marker_list[0]

        if marker == None:
            marker = self.marker_list[0]

        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.orientation.w = 1.0
        box_control.orientation.y = float(orientation[2])
        box_control.orientation.x = float(orientation[0])
        box_control.orientation.z = float(orientation[1])
        box_control.markers.append(marker)
        interactive_marker.controls.append(box_control)
        self.interactive_marker_server.applyChanges()

    def initMenu(self, dispatch_function):
        # TODO: make this init NOT hardcoded
        self.menu_handler.insert("Dispatch mission", callback=dispatch_function)

    #    entry = self.menu_handler.insert( "Drilling mode" ,callback=mode_clbk)
    #    self.menu_handler.setCheckState( entry, MenuHandler.CHECKED)

    #    entry = self.menu_handler.insert( "Painting mode" ,callback=mode_clbk)
    #    self.menu_handler.setCheckState( entry, MenuHandler.UNCHECKED)
    #    self.interactive_marker_server.applyChanges()

    def process_feedback(self, feedback):
        # TODO: HERE ADD THE MOVE OF THE SCALER POSITION RELATIVE TO THE OBJECT
        p = feedback.pose.position
        self.actual_marker_pose = feedback.pose
        print(
            feedback.marker_name
            + " is now at "
            + str(self.actual_marker_pose.position.x)
            + ", "
            + str(self.actual_marker_pose.position.y)
            + ", "
            + str(self.actual_marker_pose.position.z)
        )
        self.interactive_marker_server.applyChanges()


class ScalableInteractiveMarker(InteractiveMarkerBase):
    def __init__(self, interactive_marker, server):
        self.server = server
        self.interactive_marker = (
            interactive_marker  # the first control.marker MUST BE the marker to scale
        )

        self.resize_interactive_marker = InteractiveMarker()
        self.resize_interactive_marker.header.frame_id = (
            interactive_marker.header.frame_id
        )
        self.resize_interactive_marker.name = "resizer"

    def add_scale_control_to_interactive_marker(self, orientation=[0, 0, 1]):
        resize_control = InteractiveMarkerControl()
        resize_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        resize_control.orientation.w = 1.0
        resize_control.orientation.x = float(orientation[0])
        resize_control.orientation.y = float(orientation[2])
        resize_control.orientation.z = float(orientation[1])

        resize_arrow = Marker()
        resize_arrow.type = Marker.ARROW
        resize_arrow.pose.position.x = (
            self.interactive_marker.controls[0].markers[0].pose.position.x
            + orientation[0] * 0.5
        )
        resize_arrow.pose.position.y = (
            self.interactive_marker.controls[0].markers[0].pose.position.y
            + orientation[2] * 0.5
        )
        resize_arrow.pose.position.z = (
            self.interactive_marker.controls[0].markers[0].pose.position.z
            + orientation[1] * 0.5
        )
        resize_arrow.scale.x = 0.2
        resize_arrow.scale.y = 0.1
        resize_arrow.scale.z = 0.1
        resize_arrow.pose.orientation.w = 1.0
        resize_arrow.pose.orientation.x = float(orientation[0])
        resize_arrow.pose.orientation.y = float(orientation[2])
        resize_arrow.pose.orientation.z = float(orientation[1])
        resize_arrow.color.r = 0.09
        resize_arrow.color.g = 0.61
        resize_arrow.color.b = 0.49
        resize_arrow.color.a = 1.0
        resize_arrow.frame_locked = True

        if orientation[0] == 1 and orientation[1] == 0 and orientation[2] == 0:
            # scale x
            resize_control.name = "resize_x"
            resize_control.markers.append(resize_arrow)
            self.resize_interactive_marker.controls.append(resize_control)
            self.server.insert(
                marker=self.resize_interactive_marker, feedback_callback=self.scale_clbk
            )

        elif orientation[0] == 0 and orientation[1] == 1 and orientation[2] == 0:
            # scale y
            resize_control.name = "resize_y"
            resize_control.markers.append(resize_arrow)
            self.resize_interactive_marker.controls.append(resize_control)
            self.server.insert(
                marker=self.resize_interactive_marker, feedback_callback=self.scale_clbk
            )

        elif orientation[0] == 0 and orientation[1] == 0 and orientation[2] == 1:
            # scale z
            resize_control.name = "resize_z"
            resize_control.markers.append(resize_arrow)
            self.resize_interactive_marker.controls.append(resize_control)
            self.server.insert(
                marker=self.resize_interactive_marker, feedback_callback=self.scale_clbk
            )
        else:
            print("error, orientation MUST have only 1 axis specified")
            return

    def scale_clbk(self, feedback):
        if feedback.control_name == "resize_x":
            scale_x = 2.0 * abs(feedback.pose.position.x)
            self.interactive_marker.controls[0].markers[0].scale.x = scale_x
            self.server.insert(
                marker=self.interactive_marker, feedback_callback=self.process_feedback
            )
            self.server.applyChanges()

        elif feedback.control_name == "resize_y":
            scale_y = 2.0 * abs(feedback.pose.position.y)
            self.interactive_marker.controls[0].markers[0].scale.y = scale_y
            self.server.insert(
                marker=self.interactive_marker, feedback_callback=self.process_feedback
            )
            self.server.applyChanges()

        elif feedback.control_name == "resize_z":
            scale_z = 2.0 * abs(feedback.pose.position.z)
            self.interactive_marker.controls[0].markers[0].scale.z = scale_z
            self.server.insert(
                marker=self.interactive_marker, feedback_callback=self.process_feedback
            )
            self.server.applyChanges()

    def update_resize_arrow_pose(self, x, y, z):
        self.resize_x_im.controls[0].markers[0].pose.position.x = (
            x + self.cube_origin_to_edge_x
        )
        self.resize_x_im.controls[0].markers[0].pose.position.z = (
            z + self.cube_origin_to_edge_z
        )
        self.resize_z_im.controls[0].markers[0].pose.position.x = (
            x + self.cube_origin_to_edge_x
        )
        self.resize_z_im.controls[0].markers[0].pose.position.z = (
            z + self.cube_origin_to_edge_z
        )
        self.resize_x_server.insert(self.resize_x_im, self.resize_marker_clbk)
        self.resize_x_server.applyChanges()
        self.resize_z_server.insert(self.resize_z_im, self.resize_marker_clbk)
        self.resize_z_server.applyChanges()


def mode_clbk(feedback):
    node.get_logger().info(f"Not yet implemented, drilling is the only option")


def dispatch_clbk(feedback):
    p = feedback.pose.position
    node.get_logger().info(f"Dispatch mission, target position: \n{p}")


# ----------------------------------------------------------------------------
# in this "main" you can see an example of the use of this class

if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("rosbim_interactive_marker")

    node.get_logger().info("Created node")


    interactive_marker_base.add_new_interactive_marker(
        name="test",
        frame_id="map",
        description="without_markes",
        position=Pose(),
    )

    # interactive_marker_base.add_new_marker(Marker.CYLINDER, [0.1, 0.1, 0.1], [0.2, 0.9, 0.1, 1.0], [ 1, 0, 0])
    interactive_marker_base.add_new_marker(
        Marker.CUBE, [0.3, 0.3, 0.3], [0.2, 0.9, 0.1, 1.0]
    )

    # adding a FIXED marker to interactive marker
    interactive_marker_base.add_fixed_marker_to_interactive_marker(
        interactive_marker_base.interactive_marker_list[0],
        interactive_marker_base.marker_list[0],
    )
    # adding translation DoF on x and z to interactive marker
    interactive_marker_base.add_control_to_interactive_marker(
        interactive_marker_base.interactive_marker_list[0],
        mode=0,
        orientation=[1, 0, 0],
    )
    interactive_marker_base.add_control_to_interactive_marker(
        interactive_marker_base.interactive_marker_list[0], 0, [0, 0, 1]
    )

    interactive_marker_base.add_control_to_interactive_marker(
        interactive_marker_base.interactive_marker_list[0], 2, [0, 1, 0]
    )


    interactive_marker_base.add_menu_to_interactive_marker(
        menu_name="rosbim_mission_menu",
        marker=interactive_marker_base.marker_list[0],
        interactive_marker=interactive_marker_base.interactive_marker_list[0],
        dispatch_function=dispatch_clbk,
    )

    # # Creating a scalable marker TO TEST

    scalable_interactive_marker = ScalableInteractiveMarker(
        interactive_marker_base.interactive_marker_list[0],
        interactive_marker_base.interactive_marker_server,
    )

    scalable_interactive_marker.add_scale_control_to_interactive_marker(
        [1, 0, 0]
    )  # scale x
    scalable_interactive_marker.add_scale_control_to_interactive_marker(
        [0, 1, 0]
    )  # scale y
    scalable_interactive_marker.add_scale_control_to_interactive_marker(
        [0, 0, 1]
    )  # scale z

    interactive_marker_base.interactive_marker_server.applyChanges()

    rclpy.spin(node=node)
