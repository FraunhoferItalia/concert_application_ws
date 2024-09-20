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
import argparse
import os
import copy
import sys
from typing import Dict
import numpy as np
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import rospkg
import rclpy.parameter_service
import tf2_ros
import xml.etree.ElementTree as ET


from behavior_tree_msgs.msg import *
from behavior_tree_msgs.srv import *
from behavior_tree_msgs.action import SendBehaviorTree as SendBehaviorTreeAction


class LoadBehaviorTreeTester(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("Starting LoadBehaviorTreeTester")
        self.done = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected!")
            return

        self.get_logger().info("Goal accepted!")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result: {0}".format(result.message))
        self.done = True
        # rclpy.shutdown()

    def read_behavior_tree(self, behavior_tree_path: str):
        with open(behavior_tree_path, "r") as file:
            behavior_tree_xml = file.read()

        behavior_tree_folder = os.path.dirname(behavior_tree_path)
        et = ET.fromstring(behavior_tree_xml)
        include_tags = [c for c in et if c.tag == "include"]
        for tag in include_tags:
            et.remove(tag)
            if not tag.attrib["path"].startswith("/"):
                subtree_et = ET.fromstring(
                    self.read_behavior_tree(
                        os.path.join(behavior_tree_folder, tag.attrib["path"])
                    )
                )
                for bt in subtree_et:
                    et.insert(0, bt)
        return ET.tostring(et, encoding="utf-8", method="xml").decode("utf-8")

    # TODO: adjust this function
    def load_and_send(self, behavior_tree_path: str, input_ports: Dict[str, str] = {}, ignore_not_existing: bool=False):
        descriptor = BehaviorTreeXml()
        self._action_client = ActionClient(
            self, SendBehaviorTreeAction, "bt_action_server"
        )

        for key, value in input_ports.items():
            key_value = KeyValuePair()
            key_value.key = key
            if value is None:
                if not ignore_not_existing:
                    self.get_logger().warning(
                        f"Unable to add key-value pair to blackboard. Value at `{key}` is None"
                    )
                else:
                    continue
            key_value.value = str(value)
            descriptor.blackboard.map.append(key_value)

        descriptor.behavior_tree_xml = self.read_behavior_tree(behavior_tree_path)

        goal_msg = SendBehaviorTreeAction.Goal()
        goal_msg.tree = descriptor

        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Action Server not available, waiting again...")

        self.get_logger().info("Connected to server!")
        self.get_logger().info("Sending action...")

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.goal_accepted = False
        return


def create_dict_from_format(input_str):
    # Split the input string by whitespace and then by ":="
    return {
        k.strip(): v.strip() for k, v in (pair.split("=") for pair in input_str.split())
    }


def main(args=None):
    print("Starting behavior tree action client..")
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-b",
        "--bt",
        help="Absolute path to behaviortree to be executed",
        required=True,
    )
    parser.add_argument(
        "-p",
        "--ports",
        help="Additional bt ports",
        default="",
        required=False,
    )
    command_line_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = parser.parse_args(command_line_args)
    node = LoadBehaviorTreeTester("bt_tester")
    node.load_and_send(args.bt, create_dict_from_format(args.ports))
    while not node.done:
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
