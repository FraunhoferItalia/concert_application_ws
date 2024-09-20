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
import sys

os.environ["QT_API"] = "pyside2"

import rclpy
import rviz
from python_qt_binding.QtWidgets import QWidget, QMessageBox
from python_qt_binding.QtGui import QIcon
import yaml

from ament_index_python import get_package_share_directory
from python_qt_binding import loadUi
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from python_qt_binding import QtCore
from rclpy.executors import MultiThreadedExecutor
import threading
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo
from unique_identifier_msgs.msg import UUID
from builtin_interfaces.msg import Time
import numpy as np


class SignalsHandler(QtCore.QObject):
    service_status_signal = QtCore.Signal()

    def __init__(self):
        super(SignalsHandler, self).__init__()


class ServiceHandler(Node):
    def __init__(self):
        super().__init__("service_handler")

        package_share_directory = get_package_share_directory("concert_main_gui")

        yaml_config_path = os.path.join(
            package_share_directory,
            "config/yaml_files",
            "gui_config.yaml",
        )

        self.signals = SignalsHandler()

        yaml_config_file = open(yaml_config_path, "r")
        self.yaml_conf = yaml.safe_load(yaml_config_file)

        self.next_action_client = self.create_client(Trigger, self.yaml_conf["next_action_service"]["service_name"])
        self.stop_behavior_tree_client = self.create_client(CancelGoal, self.yaml_conf["stop_behavior_tree"]["service_name"])

        self.request = Trigger.Request()
        self.stop_request = CancelGoal.Request()

    # Call the next action of the BT
    def call_next_action(self):
        if not self.next_action_client.service_is_ready():
            try:
                self.signals.service_status_signal.emit()
            except Exception as e:
                print(e)
            return
        self.future = self.next_action_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    # Stop the actual action of BT
    def stop_behavior_tree(self):
        stamp = Time(sec=0, nanosec=0)
        goal_id = UUID(uuid=np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], dtype=np.uint8))
        goal_info = GoalInfo()
        goal_info.goal_id = goal_id
        goal_info.stamp = stamp
        self.stop_request.goal_info = goal_info
        future = self.stop_behavior_tree_client.call_async(self.stop_request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


class RqtRvizTransportWidget(QWidget):
    def __init__(self, plugin=None):
        super(RqtRvizTransportWidget, self).__init__()
        self._plugin = plugin

        self.package_share_directory = get_package_share_directory("concert_main_gui")

        ui_file = os.path.join(
            self.package_share_directory,
            "config/gui_files",
            "RvizTransportWidget.ui",
        )

        loadUi(ui_file, self)

        self.rviz_frame = rviz.VisualizationFrame()
        self.rviz_frame.setSplashPath("")
        self.rviz_frame.initialize(
            len(sys.argv),
            " ".join(sys.argv),
            os.path.join(
                self.package_share_directory,
                "config/rviz_files",
                "transport_config.rviz",
            ),
        )

        self.service_handler = ServiceHandler()

        self.nextActionButton.setStyleSheet("background-color:#179C7D")
        self.stopButton.setStyleSheet("background-color:#cc0000")

        next_icon_file = os.path.join(
            self.package_share_directory,
            "config/images",
            "next.png",
        )

        stop_icon_file = os.path.join(
            self.package_share_directory,
            "config/images",
            "stop.png",
        )

        self.nextActionButton.setIcon(QIcon(next_icon_file))
        self.stopButton.setIcon(QIcon(stop_icon_file))
        self.nextActionButton.clicked.connect(self.call_next_action_service)
        self.stopButton.clicked.connect(self.stop_behavior_tree_service)
        self.service_handler.signals.service_status_signal.connect(self.service_status_callback)
        self.rvizWidget.addWidget(self.rviz_frame)

    # Show a message if the service is not ready
    def service_status_callback(self):
        self.msg = QMessageBox()
        self.msg.setIcon(QMessageBox.Warning)
        self.msg.setText("Service Not Ready")
        self.msg.setWindowTitle("Warning")
        self.msg.setStandardButtons(QMessageBox.Cancel)
        closeTimer = QtCore.QTimer(self.msg, singleShot=True, interval=2000, 
            timeout=self.msg.close)
        closeTimer.start()    
        self.msg.exec()

   # Service to call next action from BT
    def call_next_action_service(self):
        self.service_handler.call_next_action()

    # Service to stop the actual action BT
    def stop_behavior_tree_service(self):
        self.service_handler.stop_behavior_tree()

    def shutdown(self):
        return
