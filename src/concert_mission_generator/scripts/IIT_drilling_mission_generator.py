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

from __future__ import annotations
import numpy as np

from send_behavior_tree_client import LoadBehaviorTreeTester
from rclpy.executors import MultiThreadedExecutor

from interactive_marker_base import InteractiveMarkerBase

from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import Pose, TransformStamped, Transform
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from visualization_msgs.msg import Marker
from concert_msgs.srv import MoveMarker


class ConfigurationException(BaseException):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)


def matrix_to_transform(matrix: np.object_) -> Transform:
    transform = Transform()
    transform.translation.x = matrix[0, 3]
    transform.translation.y = matrix[1, 3]
    transform.translation.z = matrix[2, 3]
    quat = R.from_matrix(matrix[:3, :3]).as_quat()
    transform.rotation.x = quat[0]
    transform.rotation.y = quat[1]
    transform.rotation.z = quat[2]
    transform.rotation.w = quat[3]
    return transform


class DrillingMissionGenerator(LoadBehaviorTreeTester):

    class Parameters:
        distance_from_wall: 1.0
        side_offset: 0.0
        angle_offset: 0.0

        def __init__(self, node: Node) -> None:
            self.distance_from_wall = (
                node.declare_parameter("distance_from_wall", 1.35)
                .get_parameter_value()
                .double_value
            )
            self.side_offset = (
                node.declare_parameter("side_offset", 0.0)
                .get_parameter_value()
                .double_value
            )
            self.angle_offset = (
                node.declare_parameter("angle_offset", 90.0)
                .get_parameter_value()
                .double_value
            )

            self.move_to_look_at_distance_offset = (
                node.declare_parameter("move_to_look_at_distance_offset", 0.15)
                .get_parameter_value()
                .double_value
            )

            node.add_on_set_parameters_callback(self.on_set_parameters_callback)

        def on_set_parameters_callback(self, params: List[Parameter]):
            all_set = True
            for parameter in params:
                if parameter.name == "distance_from_wall":
                    if parameter.type_ == Parameter.Type.DOUBLE:
                        self.distance_from_wall = parameter.value
                    else:
                        all_set = False
                if parameter.name == "side_offset":
                    if parameter.type_ == Parameter.Type.DOUBLE:
                        self.side_offset = parameter.value
                    else:
                        all_set = False
                if parameter.name == "angle_offset":
                    if parameter.type_ == Parameter.Type.DOUBLE:
                        self.angle_offset = parameter.value
                    else:
                        all_set = False
                if parameter.name == "move_to_look_at_distance_offset":
                    if parameter.type_ == Parameter.Type.DOUBLE:
                        self.move_to_look_at_distance_offset = parameter.value
                    else:
                        all_set = False

            return SetParametersResult(successful=all_set)

    def __init__(self, name: str):
        super().__init__(name)
        self.parameters = DrillingMissionGenerator.Parameters(self)

    def configure(self):
        self.get_logger().info("Creating Drilling Mission Generator")
        self.interactive_marker_base = InteractiveMarkerBase(self)
        self.subscription = self.create_subscription(
            msg_type=Pose,
            topic="/geometry_position",
            callback=self.position_callback,
            qos_profile=1,
        )
        
        self.rotate_marker_service = self.create_service(
            Trigger,
            'rotate_marker',
            self.rotate_marker_callback,
        )

        self.move_marker_service = self.create_service(
            MoveMarker,
            'move_marker',
            self.move_marker_callback,
        )

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.transforms = {}
        self.update_timer = None
        return

    def sendTransform(self, t: TransformStamped):
        self.transforms[t.child_frame_id] = t
        self.tf_broadcaster.sendTransform(list(self.transforms.values()))

    def setup_marker(self, position, orientation):
        self.interactive_marker_base.clear_interactive_marker_server()
        pose_im = Pose()
        pose_im.position.x = position[0]
        pose_im.position.y = position[1] - self.parameters.move_to_look_at_distance_offset
        pose_im.position.z = position[2]
        
        pose_im.orientation.x = orientation[0]
        pose_im.orientation.y = orientation[1]
        pose_im.orientation.z = orientation[2]
        pose_im.orientation.w = orientation[3]

        self.interactive_marker_base.add_new_interactive_marker(
            name="drilling_im",
            frame_id="floor",
            description="drilling_im",
            position=pose_im,
        )

        # Create interactive marker
        self.interactive_marker_base.add_new_marker(
            Marker.SPHERE,
            [0.33, 0.33, 0.33],
            [0.2, 0.9, 0.1, 1.0],
            [0.0, 0.0, 0.0],
            [0, 1, 0],
        )

        # adding a FIXED marker to interactive marker
        self.interactive_marker_base.add_fixed_marker_to_interactive_marker(
            self.interactive_marker_base.interactive_marker_list[0],
            self.interactive_marker_base.marker_list[0],
        )
        # adding translation DoF on x and z to interactive marker
        self.interactive_marker_base.add_control_to_interactive_marker(
            self.interactive_marker_base.interactive_marker_list[0], 0, [1, 0, 0]
        )
        self.interactive_marker_base.add_control_to_interactive_marker(
            self.interactive_marker_base.interactive_marker_list[0], 0, [0, 0, 1]
        )

        # self.interactive_marker_base.add_control_to_interactive_marker(
        #     self.interactive_marker_base.interactive_marker_list[0], 1, [0, 0, 1]
        # )

        # add menu to interactive marker, and register the mission dispatch callback
        self.interactive_marker_base.add_menu_to_interactive_marker(
            menu_name="rosbim_mission_menu",
            marker=self.interactive_marker_base.marker_list[0],
            interactive_marker=self.interactive_marker_base.interactive_marker_list[0],
            dispatch_function=self.dispatch_mission_clbk,
        )

        self.interactive_marker_base.interactive_marker_server.applyChanges()
        self.get_logger().info("Appling changes to interactive marker server")

    def get_marker_pose(self):
        position = []
        position.append(self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.position.x)
        position.append(self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.position.y + self.parameters.move_to_look_at_distance_offset)
        position.append(self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.position.z)
        orientation_x = self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.orientation.x
        orientation_y = self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.orientation.y
        orientation_z = self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.orientation.z
        orientation_w = self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.orientation.w

        marker_orientation = Rotation.from_quat([orientation_x, orientation_y, orientation_z, orientation_w])

        quaternion_orientation = [orientation_x, orientation_y, orientation_z, orientation_w]

        euler_orientation = marker_orientation.as_euler('xyz', degrees=True)

        return position, euler_orientation, quaternion_orientation


    def rotate_marker_callback(self, request, response):
        
        if self.interactive_marker_base.interactive_marker_server.get("drilling_im") == None:
            response.success=False
            response.message="marker not initialized"
            return response

        position, euler_orientation, quaternion_orientation = self.get_marker_pose()

        if euler_orientation[2]<270:
            new_euler_orientation = [euler_orientation[0], euler_orientation[1], euler_orientation[2]+90.0]
        else:
            new_euler_orientation = [euler_orientation[0], euler_orientation[1], 0.0]

        new_euler_orientation = Rotation.from_euler('xyz', new_euler_orientation, degrees=True)

        new_quaternion_orientation = new_euler_orientation.as_quat()

        self.interactive_marker_base.clear_interactive_marker_server()

        self.setup_marker(position,new_quaternion_orientation)
        self.publish_wall_origin_frame(position,new_quaternion_orientation)

        response.success=True
        response.message="marker rotated"
        return response
    
    def move_marker_callback(self, request, response):
        if self.interactive_marker_base.interactive_marker_server.get("drilling_im") == None:
            response.success=False
            return response
        
        position, euler_orientation, quaternion_orientation = self.get_marker_pose()

        if round(abs(euler_orientation[2]),0) == 0.0:
            position[0]+=request.x
        elif round(abs(euler_orientation[2]),0) == 180.0:
            position[0]-=request.x
        elif round(euler_orientation[2],0) == -90.0:
            position[1]-=request.x
        else:
            position[1]+=request.x
        position[2]+=request.z

        self.interactive_marker_base.clear_interactive_marker_server()
        self.setup_marker(position,quaternion_orientation)

        response.success=True
        return response
    
    def dispatch_mission_clbk(self, position):
        self.get_logger().info(f"Dispatching mission ...")
        self.update()
        self.get_logger().info(f"{self.mobile_base_pose}")
        self.load_and_send(
            "/concert_application_ws/src/concert_mission_generator/trees/iit/drilling.xml",
            {
                "mobile_base_pose": "; ".join([str(a) for a in self.mobile_base_pose]),
                "look_at_tf_name": "move_to_look_at",
            },
        )

    def update(self):
        self.generate_mobile_base_pose(self.interactive_marker_base.actual_marker_pose)
        self.generate_move_to_look_at(self.interactive_marker_base.actual_marker_pose)

    def generate_move_to_look_at(self, pose_im: Pose):
        im_matrix = np.eye(4)
        im_matrix[:3, 3] = np.array(
            [pose_im.position.x, pose_im.position.y, pose_im.position.z]
        )
        im_matrix[:3, :3] = R.from_quat(
            [
                pose_im.orientation.x,
                pose_im.orientation.y,
                pose_im.orientation.z,
                pose_im.orientation.w,
            ]
        ).as_matrix()

        rot_offset_matrix = np.eye(4)
        rot_offset_matrix[:3, :3] = R.from_euler("x", np.pi / 2).as_matrix()
        move_to_look_at_transform = im_matrix @ rot_offset_matrix

        mobile_base_tf = TransformStamped()
        mobile_base_tf.header.stamp = self.get_clock().now().to_msg()
        mobile_base_tf.header.frame_id = "floor"
        mobile_base_tf.child_frame_id = "move_to_look_at"
        mobile_base_tf.transform = matrix_to_transform(move_to_look_at_transform)
        self.sendTransform(mobile_base_tf)

    def generate_mobile_base_pose(self, pose_im: Pose):
        angle = R.from_quat(
            [
                pose_im.orientation.x,
                pose_im.orientation.y,
                pose_im.orientation.z,
                pose_im.orientation.w,
            ]
        ).as_euler("zxy", degrees=False)[0]
        target_x = (
            pose_im.position.x
            + self.parameters.distance_from_wall * np.sin(angle)
            + self.parameters.side_offset
        )
        target_y = pose_im.position.y - self.parameters.distance_from_wall * np.cos(
            angle
        )
        target_angle = angle + self.parameters.angle_offset * np.pi / 180

        mobile_base_tf = TransformStamped()
        mobile_base_tf.header.stamp = self.get_clock().now().to_msg()
        mobile_base_tf.header.frame_id = "floor"
        mobile_base_tf.child_frame_id = "mobile_base_pose"
        mobile_base_tf.transform.translation.x = target_x
        mobile_base_tf.transform.translation.y = target_y
        mobile_base_tf.transform.translation.z = 0.0
        q = R.from_euler("z", target_angle).as_quat()
        mobile_base_tf.transform.rotation.x = q[0]
        mobile_base_tf.transform.rotation.y = q[1]
        mobile_base_tf.transform.rotation.z = q[2]
        mobile_base_tf.transform.rotation.w = q[3]
        self.sendTransform(mobile_base_tf)
        self.mobile_base_pose = (target_x, target_y, target_angle)

    def position_callback(self, msg: Pose):
        # THESE THREE LINES MIGHT BE DANGEROUS OR USELES CHECK THEM!!!
        position = [msg.position.x*10, msg.position.y*10, msg.position.z*10]
        self.interactive_marker_base.actual_marker_pose.position.x = position[0]
        self.interactive_marker_base.actual_marker_pose.position.y = position[1]
        self.interactive_marker_base.actual_marker_pose.position.z = position[2]
        orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.setup_marker(position, orientation)
        self.publish_wall_origin_frame(position, orientation)


    def publish_wall_origin_frame(self, position, orientation):
        wall_origin_tf = TransformStamped()
        wall_origin_tf.header.stamp = self.get_clock().now().to_msg()
        wall_origin_tf.header.frame_id = "floor"
        wall_origin_tf.child_frame_id = "wall_origin"
        wall_origin_tf.transform.translation.x = position[0]
        wall_origin_tf.transform.translation.y = position[1]
        wall_origin_tf.transform.translation.z = position[2]
        wall_origin_tf.transform.rotation.w = orientation[3]
        wall_origin_tf.transform.rotation.x = orientation[0]
        wall_origin_tf.transform.rotation.y = orientation[1]
        wall_origin_tf.transform.rotation.z = orientation[2]
        wall_origin_tf.transform.rotation.w = 1.0

        # DOUBLE CHECK THESE LINES
        # self.wall_origin_matrix = np.eye(4)
        # self.wall_origin_matrix[0, 3] = position[0]
        # self.wall_origin_matrix[1, 3] = position[1]
        # self.wall_origin_matrix[2, 3] = position[2]
        self.sendTransform(wall_origin_tf)

        if self.update_timer is None:
            self.update_timer = self.create_timer(1 / 30, self.update)


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node = DrillingMissionGenerator("drilling_mission_generator")
    try:
        node.configure()
    except ConfigurationException:
        rclpy.shutdown()
        return

    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.destroy_node()
        node.get_logger().info("Keyboard interrupt, shutting down.\n")

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
