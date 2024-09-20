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
import numpy as np
import math
from typing import List
import glob
import copy

import yaml
from ament_index_python.packages import get_package_share_directory

from send_behavior_tree_client import LoadBehaviorTreeTester
from waypoint_manager_msgs.wrappers import Target, Action, Motion
from rclpy.executors import MultiThreadedExecutor

from interactive_marker_base import InteractiveMarkerBase

from scipy.spatial.transform import Rotation as R
import rclpy
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation

# import rospkg
import tf2_ros
from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose


from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from visualization_msgs.msg import Marker

from behavior_tree_msgs.msg import *
from behavior_tree_msgs.srv import *
from concert_msgs.srv import MoveMarker, MarkerService
from tf2_ros import TransformException 


# from rosbim_skills.transformations import Pose2D, Pose
# from rosbim_skills.painting_task import ArmWaypoint

# from concert_mission_generator.mission_base import InteractiveMarkerBase

from tf2_geometry_msgs import do_transform_pose

import yaml


tf_topic_frame_id = "bim_map"
MOBILE_ROBOT_DISTANCE_FROM_WALL = 2.0
SIDE_OFFSET_MOBILE_BASE = 0.0


class ConfigurationException(BaseException):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)


class DrillingMissionGenerator(LoadBehaviorTreeTester):
    def __init__(self, name: str):
        super().__init__(name)

    def configure(self):
        self.get_logger().info("Creating Drilling Mission Generator")
        self.interactive_marker_base = InteractiveMarkerBase(self)
        self.subscription = self.create_subscription(
            msg_type=Pose,
            topic="/geometry_position",
            callback=self.position_callback,
            qos_profile=1,
        )

        self.bbox_publisher = self.create_publisher(Marker, 'bbox_marker', 10)

        self.rotate_marker_service = self.create_service(
            MarkerService,
            'rotate_marker',
            self.rotate_marker_callback,
        )

        self.move_marker_service = self.create_service(
            MoveMarker,
            'move_marker',
            self.move_marker_callback,
        )

        self.spawn_marker_service = self.create_service(
            MarkerService,
            'spawn_markers',
            self.spawn_marker_callback,
        )

        self.dispatch_mission_service = self.create_service(
            Trigger,
            'dispatch_mission',
            self.dispatch_mission_service_clbk,
        )
        
        package_share_directory = get_package_share_directory("concert_mission_generator")
        yaml_config_path = os.path.join(
            package_share_directory,
            "config",
            "drill_walls.yaml",
        )

        self.pose_array = []
        self.bbox_array = []
        self.single_bbox = []
        self.current_marker_index = 0

        yaml_config_file = open(yaml_config_path, "r")
        self.yaml_conf = yaml.safe_load(yaml_config_file)

        for filename in glob.glob(os.path.join(self.yaml_conf["path"], '*.txt')):
            pose = Pose()
            with open(filename, 'r') as f:
                for l in f:
                    singlepose = l.split(";")
                pose.position.x = float(singlepose[0])
                pose.position.y = float(singlepose[1])
                pose.position.z = float(singlepose[2])
                pose.orientation.w = float(singlepose[3])
                pose.orientation.x = float(singlepose[4])
                pose.orientation.y = float(singlepose[5])
                pose.orientation.z = float(singlepose[6])

                self.single_bbox.append(singlepose[7])
                self.single_bbox.append(singlepose[8])
                self.single_bbox.append(singlepose[9])
                self.single_bbox.append(singlepose[10])
                self.single_bbox.append(singlepose[11])
                self.single_bbox.append(singlepose[12])
                singlepose.clear() 
            self.pose_array.append(pose)
            self.bbox_array.append(copy.deepcopy(self.single_bbox))
            self.single_bbox.clear()

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.aruco_tf = geometry_msgs.msg.TransformStamped()
        self.wall_origin_tf = geometry_msgs.msg.TransformStamped()
        self.drill_hole_tf = geometry_msgs.msg.TransformStamped()
        self.mobile_base_tf = geometry_msgs.msg.TransformStamped()

        return

    def setup_marker(self, position, orientation):
        self.interactive_marker_base.clear_interactive_marker_server()
        pose_im = Pose()
        pose_im.position.x = position[0]
        pose_im.position.y = position[1]
        pose_im.position.z = position[2]

        pose_im.orientation.x = orientation[0]
        pose_im.orientation.y = orientation[1]
        pose_im.orientation.z = orientation[2]
        pose_im.orientation.w = orientation[3]

        quaternion_orientation = Rotation.from_quat(orientation)

        euler_orientation = quaternion_orientation.as_euler('xyz', degrees=True)

        self.interactive_marker_base.add_new_interactive_marker(
            name="drilling_im",
            frame_id=tf_topic_frame_id,
            description="drilling_im",
            position=pose_im,
        )

        # Create interactive marker
        self.interactive_marker_base.add_new_marker(
            Marker.ARROW,
            [0.2, 0.1, 0.1],
            [0.2, 0.9, 0.1, 1.0],
            [0.0, -0.1, 0.0],
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

        #self.interactive_marker_base.add_control_to_interactive_marker(
        #    self.interactive_marker_base.interactive_marker_list[0], 1, [0, 0, 1]
        #)

        # add menu to interactive marker, and register the mission dispatch callback
        self.interactive_marker_base.add_menu_to_interactive_marker(
            menu_name="rosbim_mission_menu",
            marker=self.interactive_marker_base.marker_list[0],
            interactive_marker=self.interactive_marker_base.interactive_marker_list[0],
            dispatch_function=self.dispatch_mission_clbk,
        )

        self.interactive_marker_base.interactive_marker_server.applyChanges()
        self.get_logger().info("Appling changes to interactive marker server")

        self.publish_drill_hole_frame(self.interactive_marker_base.actual_marker_pose)
        return
    
    def rotate_marker_callback(self, request, response):
        
        #if self.interactive_marker_base.interactive_marker_server.get("drilling_im") == None:
        #    response.success=False
        #    response.message="marker not initialized"
        #    return response

        position = []
        position.append(self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.position.x)
        position.append(self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.position.y)
        position.append(self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.position.z)
        orientation_x = self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.orientation.x
        orientation_y = self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.orientation.y
        orientation_z = self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.orientation.z
        orientation_w = self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.orientation.w

        quaternion_orientation = Rotation.from_quat([orientation_x, orientation_y, orientation_z, orientation_w])

        euler_orientation = quaternion_orientation.as_euler('xyz', degrees=True)

        if euler_orientation[2]<270:
            euler_orientation = [euler_orientation[0], euler_orientation[1], euler_orientation[2]+90.0]
        else:
            euler_orientation = [euler_orientation[0], euler_orientation[1], 0.0]

        new_euler_orientation = Rotation.from_euler('xyz', euler_orientation, degrees=True)

        new_quaternion_orientation = new_euler_orientation.as_quat()

        self.interactive_marker_base.clear_interactive_marker_server()

        self.publish_wall_origin_frame(position,new_quaternion_orientation)
        self.setup_marker(position,new_quaternion_orientation)

        response.success = True
        response.orientation= euler_orientation[2]
        return response
    
    def move_marker_callback(self, request, response):
        if self.interactive_marker_base.interactive_marker_server.get("drilling_im") == None:
            response.success=False
            return response
        
        position = []
        orientation = []
        position.append(self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.position.x)
        position.append(self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.position.y)
        position.append(self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.position.z)
        orientation.append(self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.orientation.x)
        orientation.append(self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.orientation.y)
        orientation.append(self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.orientation.z)
        orientation.append(self.interactive_marker_base.interactive_marker_server.get("drilling_im").pose.orientation.w)

        quaternion_orientation = Rotation.from_quat([orientation[0], orientation[1], orientation[2], orientation[3]])

        euler_orientation = quaternion_orientation.as_euler('xyz', degrees=True)

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
        self.setup_marker(position,orientation)

        response.success=True
        return response

    def dispatch_mission_service_clbk(self, request,response):
        path_to_aruco_yaml_file = (
            "/concert_application_ws/src/concert_mission_generator/config/"
            + "aruco_look_up_table.yaml"
        )

        aruco_id = self.aruco_look_up(path_to_aruco_yaml_file)
        self.get_logger().info(f"\nClosest ARUCO ID: {aruco_id}\n")
        #self.publish_drill_hole_frame(self.interactive_marker_base.actual_marker_pose)
        mobile_base_pose = self.generate_mobile_base_pose(
            self.interactive_marker_base.actual_marker_pose
        )
        self.get_logger().info(
            f"\nMobile base pose: \nx:{mobile_base_pose[0]}\ny:{mobile_base_pose[1]}\ntheta:{mobile_base_pose[2]}\n"
        )
        mobile_base_pose_approach = [
            mobile_base_pose[0],
            mobile_base_pose[1],
            mobile_base_pose[2],
        ]
        arm_position = self.generate_arm_pose(
            self.interactive_marker_base.actual_marker_pose
        )
        input_ports = {
            "aruco_id": aruco_id,
            "mobile_base_pose_approach": f"{mobile_base_pose_approach[0]};{mobile_base_pose_approach[1]};{mobile_base_pose_approach[2]}",
            "mobile_base_pose": f"{mobile_base_pose[0]};{mobile_base_pose[1]};{mobile_base_pose[2]}",
            "waypoint_msgs_drilling": arm_position,
        }

        self.load_and_send(
            "/concert_application_ws/src/concert_mission_generator/trees/drilling.xml",
            input_ports,
            ignore_not_existing=True
        )

        response.success=True
        return response
    
    def dispatch_mission_clbk(self, position):
        path_to_aruco_yaml_file = (
            "/concert_application_ws/src/concert_mission_generator/config/"
            + "aruco_look_up_table.yaml"
        )

        aruco_id = self.aruco_look_up(path_to_aruco_yaml_file)
        self.get_logger().info(f"\nClosest ARUCO ID: {aruco_id}\n")
        #self.publish_drill_hole_frame(self.interactive_marker_base.actual_marker_pose)
        mobile_base_pose = self.generate_mobile_base_pose(
            self.interactive_marker_base.actual_marker_pose
        )
        self.get_logger().info(
            f"\nMobile base pose: \nx:{mobile_base_pose[0]}\ny:{mobile_base_pose[1]}\ntheta:{mobile_base_pose[2]}\n"
        )
        mobile_base_pose_approach = [
            mobile_base_pose[0],
            mobile_base_pose[1],
            mobile_base_pose[2],
        ]
        arm_position = self.generate_arm_pose(
            self.interactive_marker_base.actual_marker_pose
        )
        input_ports = {
            "aruco_id": aruco_id,
            "mobile_base_pose_approach": f"{mobile_base_pose_approach[0]};{mobile_base_pose_approach[1]};{mobile_base_pose_approach[2]}",
            "mobile_base_pose": f"{mobile_base_pose[0]};{mobile_base_pose[1]};{mobile_base_pose[2]}",
            "waypoint_msgs_drilling": arm_position,
        }

        sim_suffix = (
            "_sim"
            if self.get_parameter("use_sim_time").get_parameter_value().bool_value
            else ""
        )

        self.load_and_send(
            f"/concert_application_ws/src/concert_mission_generator/trees/drilling{sim_suffix}.xml",
            input_ports,
            ignore_not_existing=True,
        )
        return

    def publish_drill_hole_frame(self, pose_im: geometry_msgs.msg.Pose):
        self.drill_hole_tf.header.stamp = self.get_clock().now().to_msg()
        self.drill_hole_tf.header.frame_id = "bim_map"
        self.drill_hole_tf.child_frame_id = "drill_hole"
        
        self.drill_hole_tf.transform.translation.x = pose_im.position.x
        self.drill_hole_tf.transform.translation.y = pose_im.position.y
        self.drill_hole_tf.transform.translation.z = pose_im.position.z

        q = [
            pose_im.orientation.x,
            pose_im.orientation.y,
            pose_im.orientation.z,
            pose_im.orientation.w,
        ]

        r_im = R.from_quat(q)
        orientation_offset = [
            0.0,
            0.7071068,
            0.7071068,
            0.0,
        ]  # this changes the orientation of interactive marker wrt base with z outward (aruco-detection like)
        r_offset = R.from_quat(orientation_offset)
        r = r_im * r_offset
        q = r.as_quat()

        self.drill_hole_tf.transform.rotation.x = q[0]
        self.drill_hole_tf.transform.rotation.y = q[1]
        self.drill_hole_tf.transform.rotation.z = q[2]
        self.drill_hole_tf.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(self.drill_hole_tf)
        return

    def generate_mobile_base_pose(self, pose_im: Pose):
        self.get_logger().info("Generate mobile base pose...")

        mobile_base_pose = Pose()

        r = R.from_quat(
            [
                pose_im.orientation.x,
                pose_im.orientation.y,
                pose_im.orientation.z,
                pose_im.orientation.w,
            ]
        )
        zxy = r.as_euler("zxy", degrees=False)
        angle = zxy[0]  # rotation arround Z axis
        mobile_base_pose.position.x = (
            pose_im.position.x
            + MOBILE_ROBOT_DISTANCE_FROM_WALL * np.sin(angle)
            + SIDE_OFFSET_MOBILE_BASE
        )
        mobile_base_pose.position.y = (
            pose_im.position.y - MOBILE_ROBOT_DISTANCE_FROM_WALL * np.cos(angle)
        )
        mobile_base_pose.position.z = 0.0  # to be transformed on map floor

        self.mobile_base_tf.header.stamp = self.get_clock().now().to_msg()
        self.mobile_base_tf.header.frame_id = "bim_map"
        self.mobile_base_tf.child_frame_id = "mobile_base_pose"
        self.mobile_base_tf.transform.translation.x = mobile_base_pose.position.x
        self.mobile_base_tf.transform.translation.y = mobile_base_pose.position.y
        self.mobile_base_tf.transform.translation.z = mobile_base_pose.position.z

        orientation_offset = [
            0.0,
            0.0,
            0.7071068,
            0.7071068,
        ]  # this rotate mobile base frame with mobile_base.x the orientation of mobile base marker wrt with z outward (aruco-detection like)
        r_offset = R.from_quat(orientation_offset)
        r = r * r_offset
        q = r.as_quat()
        self.mobile_base_tf.transform.rotation.x = q[0]
        self.mobile_base_tf.transform.rotation.y = q[1]
        self.mobile_base_tf.transform.rotation.z = q[2]
        self.mobile_base_tf.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(self.mobile_base_tf)

        mobile_base_orientation_r = R.from_quat(q)

        theta = mobile_base_orientation_r.as_euler("zyx", degrees=False)[0]

        return (
            float(mobile_base_pose.position.x),
            float(mobile_base_pose.position.y),
            float(theta),
        )

    def generate_arm_pose(self, pose: Pose) -> String:
        """_summary_

        Args:
            pose (Pose): pose of the interactive marker, needed to create the relative movement of the arm

        Returns:
            String: string in yaml format to send to the waypoint_manager
        """

        q = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]

        r_pose = R.from_quat(q)
        orientation_offset = [
            1.0,
            0.0,
            0.0,
            0.0,
        ]  # this changes the orientation of the desired orientation of ee with z in opposite direction of arucocode detection
        r_offset = R.from_quat(orientation_offset)
        r = r_pose * r_offset
        q_with_offset = r.as_quat()

        target_frame = "aruco_frame_"
        source_frame = "drill_hole"

        self.tf_buffer.can_transform(target_frame, source_frame, self.get_clock().now())
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            self.get_logger().info(
                f"Faild to lookup_transform {target_frame} to {source_frame}"
            )
            return ""

        target_pose = transform_stamped.transform
        target_orientation_r = R.from_quat(
            [
                float(target_pose.rotation.x),
                float(target_pose.rotation.y),
                float(target_pose.rotation.z),
                float(target_pose.rotation.w),
            ]
        )
        r_offset1 = R.from_quat([0.0, 1.0, 0.0, 0.0])
        r_offset2 = R.from_quat([0.0, 0.0, 0.5372996, 0.8433914])
        target_orientation_r = target_orientation_r * r_offset1 * r_offset2
        target_orientation_q = target_orientation_r.as_quat()

        target_predrill = Target(
            name="drilling/drill/approach/predrill",
            frame_id="aruco_frame_",
            tip_frame="pino_ee_A",
            seed=[0.0, 0.75, 1.57, -0.75, -1.1, 0.0],
            scale_type=Target.ScaleType.RELATIVE,
            space_type=Target.SpaceType.CARTESIAN,
            target=[
                float(target_pose.translation.x),
                float(target_pose.translation.y),
                float(target_pose.translation.z + 0.15),
                float(target_orientation_q[0]),
                float(target_orientation_q[1]),
                float(target_orientation_q[2]),
                float(target_orientation_q[3]),
            ],
        )

        approach_motion = Motion(
            name="drilling/approach", type=Motion.Type.JOINT, targets=[target_predrill]
        )
        approach_action = Action(
            name="drilling/approach", group_name="pino_arm", motions=[approach_motion]
        )
        actions: List[Action] = [
            approach_action,
        ]

        return yaml.safe_dump(
            [a.to_dict() for a in actions], sort_keys=False, default_flow_style=None
        )

    def position_callback(self, msg):
        # TODO: test if it works or other possible way to fix the problem that without moving im "actual_marker_pose" does not update
        self.interactive_marker_base.actual_marker_pose.position.x = msg.position.x
        self.interactive_marker_base.actual_marker_pose.position.y = msg.position.y
        self.interactive_marker_base.actual_marker_pose.position.z = msg.position.z

        position = [msg.position.x, msg.position.y, msg.position.z]
        orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        self.publish_wall_origin_frame(position, orientation)
        self.setup_marker(position, orientation)
        return
    
    def spawn_bbox(self, minx, miny, minz, maxx, maxy, maxz) :
        marker = Marker()

        marker.action = Marker.DELETEALL
        self.bbox_publisher.publish(marker)

        marker.header.frame_id = "/wall_origin"
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD

        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        first_point = Point()
        first_point.x = minx
        first_point.y = miny
        first_point.z = minz

        second_point = Point()
        second_point.x = minx
        second_point.y = miny
        second_point.z = maxz

        third_point = Point()
        third_point.x = maxx
        third_point.y = miny
        third_point.z = maxz

        fourth_point = Point()
        fourth_point.x = maxx
        fourth_point.y = miny
        fourth_point.z = minz

        fifth_point = Point()
        fifth_point.x = minx
        fifth_point.y = maxy
        fifth_point.z = minz

        sixth_point = Point()
        sixth_point.x = minx
        sixth_point.y = maxy
        sixth_point.z = maxz

        seventh_point = Point()
        seventh_point.x = maxx
        seventh_point.y = maxy
        seventh_point.z = maxz

        eighth_point = Point()
        eighth_point.x = maxx
        eighth_point.y = maxy
        eighth_point.z = minz

        marker.points.append(first_point)
        marker.points.append(second_point)

        marker.points.append(first_point)
        marker.points.append(fourth_point)

        marker.points.append(first_point)
        marker.points.append(fifth_point)

        marker.points.append(fifth_point)
        marker.points.append(sixth_point)

        marker.points.append(fifth_point)
        marker.points.append(eighth_point)

        marker.points.append(second_point)
        marker.points.append(sixth_point)

        marker.points.append(sixth_point)
        marker.points.append(seventh_point)

        marker.points.append(seventh_point)
        marker.points.append(eighth_point)

        marker.points.append(second_point)
        marker.points.append(third_point)

        marker.points.append(fourth_point)
        marker.points.append(eighth_point)
       
        marker.points.append(third_point)
        marker.points.append(fourth_point)

        marker.points.append(third_point)
        marker.points.append(seventh_point)

        # Publish the Marker
        self.bbox_publisher.publish(marker)

    
    def spawn_marker_callback(self, request, response):
        position = [self.pose_array[self.current_marker_index].position.x, self.pose_array[self.current_marker_index].position.y, self.pose_array[self.current_marker_index].position.z]
        orientation = [self.pose_array[self.current_marker_index].orientation.x, self.pose_array[self.current_marker_index].orientation.y, self.pose_array[self.current_marker_index].orientation.z, self.pose_array[self.current_marker_index].orientation.w]
        self.publish_wall_origin_frame(position, orientation)
        self.setup_marker(position, orientation)

        quaternion_orientation = Rotation.from_quat([orientation[0], orientation[1], orientation[2], orientation[3]])
        euler_orientation = quaternion_orientation.as_euler('xyz', degrees=True)
        response.success = True
        response.orientation = euler_orientation[2]

        minx = float(self.bbox_array[self.current_marker_index][0]) 
        miny = float(self.bbox_array[self.current_marker_index][1]) 
        minz = float(self.bbox_array[self.current_marker_index][2]) 
        maxx = float(self.bbox_array[self.current_marker_index][3]) 
        maxy = float(self.bbox_array[self.current_marker_index][4]) 
        maxz = float(self.bbox_array[self.current_marker_index][5])


        self.spawn_bbox(minx, miny, minz, maxx, maxy, maxz)

        self.current_marker_index +=1 
        if len(self.pose_array) == self.current_marker_index:
            self.current_marker_index = 0

        return response


    def publish_wall_origin_frame(self, position, orientation):
        self.wall_origin_tf.header.stamp = self.get_clock().now().to_msg()
        self.wall_origin_tf.header.frame_id = "bim_map"
        self.wall_origin_tf.child_frame_id = "wall_origin"
        self.wall_origin_tf.transform.translation.x = position[0]
        self.wall_origin_tf.transform.translation.y = position[1]
        self.wall_origin_tf.transform.translation.z = position[2]
        self.wall_origin_tf.transform.rotation.w = orientation[3]
        self.wall_origin_tf.transform.rotation.x = orientation[0]
        self.wall_origin_tf.transform.rotation.y = orientation[1]
        self.wall_origin_tf.transform.rotation.z = orientation[2]
        self.publish_aruco_drilling_task_frame()

        self.tf_broadcaster.sendTransform(self.wall_origin_tf)

    def publish_aruco_drilling_task_frame(self):
        self.aruco_tf.header.stamp = self.get_clock().now().to_msg()
        self.aruco_tf.header.frame_id = "wall_origin"
        self.aruco_tf.child_frame_id = "aruco_frame_"
        # TODO: to be substituted with aruco positon on wall and later to be calibrated by MM
        self.aruco_tf.transform.translation.x = 0.01
        self.aruco_tf.transform.translation.y = (
            -0.30
        )  # -0.055 with new position of wall (in front of arena big door)
        self.aruco_tf.transform.translation.z = -0.15
        # q = [0.0, 0.0, 0.0, 1.0]
        # q = [0.0, 0.7071068, 0.7071068, 0.0]
        q = [
            0.7071068,
            0.0,
            0.0,
            0.7071068,
        ]  # TODO: hardcoded orientation...
        self.aruco_tf.transform.rotation.x = q[0]
        self.aruco_tf.transform.rotation.y = q[1]
        self.aruco_tf.transform.rotation.z = q[2]
        self.aruco_tf.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(self.aruco_tf)
        return

    def aruco_look_up(self, aruco_look_up_table_path: String) -> int:
        """function that returns the closest aruco id to the IM, checking a file(?) that has ARUCO_ID - POSITION (wrt map)"""

        with open(aruco_look_up_table_path, "r") as file:
            aruco_id_positions = yaml.safe_load(
                file
            )  # to be filled looking in the file

        min_distance = None
        aruco_id_min_distance = -1
        im_pose = self.interactive_marker_base.actual_marker_pose

        for id in aruco_id_positions:
            distance = math.sqrt(
                (im_pose.position.x - aruco_id_positions[id][0]) ** 2
                + (im_pose.position.y - aruco_id_positions[id][1]) ** 2
                + (im_pose.position.z - aruco_id_positions[id][2]) ** 2
            )
            if min_distance == None:
                min_distance = distance
                aruco_id_min_distance = id
            elif distance < min_distance:
                min_distance = distance
                aruco_id_min_distance = id
        return int(aruco_id_min_distance.split("_")[1])


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
