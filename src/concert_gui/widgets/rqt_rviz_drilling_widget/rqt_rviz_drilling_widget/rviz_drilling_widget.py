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
from python_qt_binding.QtWidgets import QWidget, QMessageBox, QShortcut
from python_qt_binding.QtGui import QIcon, QKeySequence
import yaml

from ament_index_python import get_package_share_directory
from python_qt_binding import loadUi
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from python_qt_binding import QtCore
from rclpy.executors import MultiThreadedExecutor
import threading
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo
from concert_msgs.srv import MoveMarker, MarkerService
from unique_identifier_msgs.msg import UUID
from builtin_interfaces.msg import Time
import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class SignalsHandler(QtCore.QObject):
    service_status_signal = QtCore.Signal()
    marker_position_signal = QtCore.Signal()

    def __init__(self):
        super(SignalsHandler, self).__init__()


class ServiceHandler(Node):
    def __init__(self):
        super().__init__("service_handler")

        package_share_directory = get_package_share_directory("concert_main_gui")

        self.rviz_config = (
            self.declare_parameter("rviz_file", "drilling")
            .get_parameter_value()
            .string_value
        )

        yaml_config_path = os.path.join(
            package_share_directory,
            "config/yaml_files",
            "gui_config.yaml",
        )

        self.signals = SignalsHandler()

        yaml_config_file = open(yaml_config_path, "r")
        self.yaml_conf = yaml.safe_load(yaml_config_file)

        self.next_action_client = self.create_client(
            Trigger, self.yaml_conf["next_action_service"]["service_name"]
        )
        self.stop_behavior_tree_client = self.create_client(
            CancelGoal, self.yaml_conf["stop_behavior_tree"]["service_name"]
        )

        self.rotate_marker_client = self.create_client(
            MarkerService, self.yaml_conf["rotate_marker_service"]["service_name"]
        )
        self.move_marker_client = self.create_client(
            MoveMarker, self.yaml_conf["move_marker_service"]["service_name"]
        )

        self.spawn_marker_client = self.create_client(MarkerService, "spawn_markers")
        self.dispatch_mission_client = self.create_client(Trigger, "dispatch_mission")

        self.next_action_request = Trigger.Request()
        self.stop_request = CancelGoal.Request()
        self.rotate_marker_request = MarkerService.Request()
        self.spawn_marker_request = MarkerService.Request()
        self.dispatch_mission_request = Trigger.Request()
        self.move_marker_request = MoveMarker.Request()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    # Call the next action of the BT
    def call_next_action(self):
        if not self.next_action_client.service_is_ready():
            try:
                self.signals.service_status_signal.emit()
            except Exception as e:
                print(e)
            return
        self.future = self.next_action_client.call_async(self.next_action_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    # Stop the actual action of BT
    def stop_behavior_tree(self):
        stamp = Time(sec=0, nanosec=0)
        goal_id = UUID(
            uuid=np.array(
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.uint8
            )
        )
        goal_info = GoalInfo()
        goal_info.goal_id = goal_id
        goal_info.stamp = stamp

        self.stop_request.goal_info = goal_info
        future = self.stop_behavior_tree_client.call_async(self.stop_request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    # Rotate the spawned marker 90 degrees
    def rotate_marker(self):
        if not self.rotate_marker_client.service_is_ready():
            try:
                self.signals.service_status_signal.emit()
            except Exception as e:
                print(e)
            return
        self.future = self.rotate_marker_client.call_async(self.rotate_marker_request)
        rclpy.spin_until_future_complete(self, self.future)
        self.signals.marker_position_signal.emit()
        return self.future.result()

    # Translate the marker positon
    def move_marker(self, x, z):
        if not self.move_marker_client.service_is_ready():
            try:
                self.signals.service_status_signal.emit()
            except Exception as e:
                print(e)
            return
        self.move_marker_request.x = x
        self.move_marker_request.z = z
        self.future = self.move_marker_client.call_async(self.move_marker_request)
        rclpy.spin_until_future_complete(self, self.future)
        self.signals.marker_position_signal.emit()
        return self.future.result()

    # Spawn the interactive marker
    def call_spawn_marker(self):
        self.future = self.spawn_marker_client.call_async(self.spawn_marker_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    # Start the mission BT
    def call_dispatch_mission(self):
        self.future = self.dispatch_mission_client.call_async(
            self.dispatch_mission_request
        )
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


class RqtRvizDrillingWidget(QWidget):
    def __init__(self, plugin=None):
        super(RqtRvizDrillingWidget, self).__init__()
        self._plugin = plugin

        self.package_share_directory = get_package_share_directory("concert_main_gui")

        ui_file = os.path.join(
            self.package_share_directory,
            "config/gui_files",
            "RvizDrillingWidget.ui",
        )

        loadUi(ui_file, self)

        self.service_handler = ServiceHandler()
        self.rviz_frame = rviz.VisualizationFrame()
        self.rviz_frame.setSplashPath("")
        self.rviz_frame.initialize(
            len(sys.argv),
            " ".join(sys.argv),
            os.path.join(
                self.package_share_directory,
                "config/rviz_files",
                self.service_handler.rviz_config + "_config.rviz",
            ),
        )

        self.nextActionButton.setStyleSheet("background-color:#179C7D")
        self.stopButton.setStyleSheet("background-color:#cc0000")
        self.moveUpButton.setStyleSheet("background-color:#179C7D")
        self.moveDownButton.setStyleSheet("background-color:#179C7D")
        self.moveRightButton.setStyleSheet("background-color:#179C7D")
        self.moveLeftButton.setStyleSheet("background-color:#179C7D")
        self.rotateButton.setStyleSheet("background-color:#179C7D")
        self.defaultViewButton.setStyleSheet("background-color:#179C7D")
        self.dispatchButton.setStyleSheet("background-color:#179C7D")
        self.markerViewButton.setStyleSheet("background-color:#179C7D")
        self.spawnButton.setStyleSheet("background-color:#179C7D")
        self.topViewButton.setStyleSheet("background-color:#179C7D")

        self.nextActionButton.clicked.connect(self.call_next_action_service)
        self.stopButton.clicked.connect(self.stop_behavior_tree_service)
        self.rotateButton.clicked.connect(self.rotate_marker_service)
        self.moveUpButton.clicked.connect(lambda: self.move_marker_service(0.0, 0.05))
        self.moveDownButton.clicked.connect(
            lambda: self.move_marker_service(0.0, -0.05)
        )
        self.moveLeftButton.clicked.connect(
            lambda: self.move_marker_service(-0.05, 0.0)
        )
        self.moveRightButton.clicked.connect(
            lambda: self.move_marker_service(0.05, 0.0)
        )
        self.spawnButton.clicked.connect(self.call_spawn_marker_service)
        self.dispatchButton.clicked.connect(self.call_dispatch_mission_service)
        self.topViewButton.clicked.connect(self.switch_top_view)
        self.defaultViewButton.clicked.connect(self.switch_default_view)
        self.markerViewButton.clicked.connect(
            lambda: self.view_on_marker(self.marker_orientation)
        )

        self.shortcut_moveUp = QShortcut(QKeySequence(QtCore.Qt.Key_Up), self)
        self.shortcut_moveUp.activated.connect(
            lambda: self.move_marker_service(0.0, 0.05)
        )
        self.shortcut_moveDown = QShortcut(QKeySequence(QtCore.Qt.Key_Down), self)
        self.shortcut_moveDown.activated.connect(
            lambda: self.move_marker_service(0.0, -0.05)
        )
        self.shortcut_moveLeft = QShortcut(QKeySequence(QtCore.Qt.Key_Left), self)
        self.shortcut_moveLeft.activated.connect(
            lambda: self.move_marker_service(-0.05, 0.0)
        )
        self.shortcut_moveRight = QShortcut(QKeySequence(QtCore.Qt.Key_Right), self)
        self.shortcut_moveRight.activated.connect(
            lambda: self.move_marker_service(0.05, 0.0)
        )
        self.shortcut_rotateMarker = QShortcut(QKeySequence("R"), self)
        self.shortcut_rotateMarker.activated.connect(self.rotate_marker_service)
        self.shortcut_dispatchMission = QShortcut(QKeySequence("D"), self)
        self.shortcut_dispatchMission.activated.connect(
            self.call_dispatch_mission_service
        )
        self.shortcut_spawnMarker = QShortcut(QKeySequence("S"), self)
        self.shortcut_spawnMarker.activated.connect(self.call_spawn_marker_service)
        self.shortcut_nextAction = QShortcut(QKeySequence("N"), self)
        self.shortcut_nextAction.activated.connect(self.call_next_action_service)
        self.shortcut_stopAction = QShortcut(QKeySequence("P"), self)
        self.shortcut_stopAction.activated.connect(self.stop_behavior_tree_service)
        self.shortcut_topView = QShortcut(QKeySequence("T"), self)
        self.shortcut_topView.activated.connect(self.switch_top_view)
        self.shortcut_defaultView = QShortcut(QKeySequence("F"), self)
        self.shortcut_defaultView.activated.connect(self.switch_default_view)
        self.shortcut_markerView = QShortcut(QKeySequence("M"), self)
        self.shortcut_markerView.activated.connect(
            lambda: self.view_on_marker(self.marker_orientation)
        )

        self.nextActionButton.setEnabled(False)
        self.stopButton.setEnabled(False)
        self.rotateButton.setEnabled(False)
        self.moveUpButton.setEnabled(False)
        self.moveDownButton.setEnabled(False)
        self.moveLeftButton.setEnabled(False)
        self.moveRightButton.setEnabled(False)
        self.dispatchButton.setEnabled(False)
        self.lcdXposition.setEnabled(False)
        self.lcdYposition.setEnabled(False)

        self.marker_orientation = 0

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

        moveup_icon_file = os.path.join(
            self.package_share_directory,
            "config/images",
            "moveup.png",
        )

        movedown_icon_file = os.path.join(
            self.package_share_directory,
            "config/images",
            "movedown.png",
        )

        moveleft_icon_file = os.path.join(
            self.package_share_directory,
            "config/images",
            "moveleft.png",
        )

        moveright_icon_file = os.path.join(
            self.package_share_directory,
            "config/images",
            "moveright.png",
        )

        rotate_icon_file = os.path.join(
            self.package_share_directory,
            "config/images",
            "rotate.png",
        )

        marker_view_icon_file = os.path.join(
            self.package_share_directory,
            "config/images",
            "marker_view.png",
        )

        default_view_icon_file = os.path.join(
            self.package_share_directory,
            "config/images",
            "default_view.png",
        )

        top_view_icon_file = os.path.join(
            self.package_share_directory,
            "config/images",
            "top_view.png",
        )

        spawn_marker_icon_file = os.path.join(
            self.package_share_directory,
            "config/images",
            "spawn_marker.png",
        )

        dispatch_mission_icon_file = os.path.join(
            self.package_share_directory,
            "config/images",
            "dispatch_mission.png",
        )

        self.nextActionButton.setIcon(QIcon(next_icon_file))
        self.stopButton.setIcon(QIcon(stop_icon_file))
        self.moveUpButton.setIcon(QIcon(moveup_icon_file))
        self.moveDownButton.setIcon(QIcon(movedown_icon_file))
        self.moveLeftButton.setIcon(QIcon(moveleft_icon_file))
        self.moveRightButton.setIcon(QIcon(moveright_icon_file))
        self.rotateButton.setIcon(QIcon(rotate_icon_file))
        self.defaultViewButton.setIcon(QIcon(default_view_icon_file))
        self.dispatchButton.setIcon(QIcon(dispatch_mission_icon_file))
        self.markerViewButton.setIcon(QIcon(marker_view_icon_file))
        self.spawnButton.setIcon(QIcon(spawn_marker_icon_file))
        self.topViewButton.setIcon(QIcon(top_view_icon_file))

        self.spawned_marker = False

        self.service_handler.signals.service_status_signal.connect(
            self.service_status_callback
        )
        self.service_handler.signals.marker_position_signal.connect(
            self.change_lcd_values(0.0, 0.0)
        )

        self.rvizWidget.addWidget(self.rviz_frame)

    # Show a message if the service is not ready
    def service_status_callback(self):
        self.msg = QMessageBox()
        self.msg.setIcon(QMessageBox.Warning)
        self.msg.setText("Service Not Ready")
        self.msg.setWindowTitle("Warning")
        self.msg.setStandardButtons(QMessageBox.Cancel)
        closeTimer = QtCore.QTimer(
            self.msg, singleShot=True, interval=2000, timeout=self.msg.close
        )
        closeTimer.start()
        self.msg.exec()

    # Show the translation of the marker in lcd displays
    def change_lcd_values(self, x, y):
        self.lcdXposition.display(x)
        self.lcdXposition.repaint()
        self.lcdYposition.display(y)
        self.lcdYposition.repaint()

    # Change the current view to focus on the markrer
    def view_on_marker(self, marker_orient):
        if not self.spawned_marker:
            return
        vc = self.rviz_frame.getManager().getViewManager().getCurrent()
        self.marker_orientation = marker_orient
        if vc.getClassId() == "rviz_default_plugins/Orbit":
            vc.subProp("Target Frame").setValue("aruco_frame_")
            vc.subProp("Pitch").setValue(0)
            vc.subProp("Focal Point").subProp("X").setValue(0.073516)
            vc.subProp("Focal Point").subProp("Y").setValue(-0.60485)
            vc.subProp("Focal Point").subProp("Z").setValue(0.33589)

            if round(abs(marker_orient), 0) == 0.0:
                vc.subProp("Distance").setValue(2.28)
                vc.subProp("Yaw").setValue(4.69)
            elif round(abs(marker_orient), 0) == 90.0:
                vc.subProp("Distance").setValue(2.52)
                vc.subProp("Yaw").setValue(0.23)
            elif round(abs(marker_orient), 0) == 180.0:
                vc.subProp("Distance").setValue(3.5)
                vc.subProp("Yaw").setValue(1.62)
            elif round(abs(marker_orient), 0) == 270.0:
                vc.subProp("Distance").setValue(2.52)
                vc.subProp("Yaw").setValue(2.9)

    # Change the actual view to top-view
    def switch_top_view(self):
        vc = self.rviz_frame.getManager().getViewManager().getCurrent()
        vc.subProp("Distance").setValue(36.14)
        vc.subProp("Target Frame").setValue("map")
        vc.subProp("Yaw").setValue(4.71)
        vc.subProp("Pitch").setValue(1.57)
        vc.subProp("Focal Point").subProp("X").setValue(8.76)
        vc.subProp("Focal Point").subProp("Y").setValue(-9.51)
        vc.subProp("Focal Point").subProp("Z").setValue(0.32)

    # Change the current view to default view
    def switch_default_view(self):
        vc = self.rviz_frame.getManager().getViewManager().getCurrent()
        vc.subProp("Distance").setValue(18.40)
        vc.subProp("Target Frame").setValue("map")
        vc.subProp("Yaw").setValue(0.01)
        vc.subProp("Pitch").setValue(0.575)
        vc.subProp("Focal Point").subProp("X").setValue(2.44)
        vc.subProp("Focal Point").subProp("Y").setValue(-2.70)
        vc.subProp("Focal Point").subProp("Z").setValue(-1.61)

    # Call the service that spawn the marker and activate marker-movements buttons
    def call_spawn_marker_service(self):
        response = self.service_handler.call_spawn_marker()
        self.rotateButton.setEnabled(True)
        self.moveUpButton.setEnabled(True)
        self.moveDownButton.setEnabled(True)
        self.moveLeftButton.setEnabled(True)
        self.moveRightButton.setEnabled(True)
        self.dispatchButton.setEnabled(True)
        self.lcdXposition.setEnabled(True)
        self.lcdYposition.setEnabled(True)
        self.change_lcd_values(0.0, 0.0)
        self.spawned_marker = True
        self.marker_orientation = response.orientation

    # Service to call next action from BT
    def call_next_action_service(self):
        if not self.spawned_marker:
            return
        self.service_handler.call_next_action()

    # Service to dispatch the mission 
    def call_dispatch_mission_service(self):
        if not self.spawned_marker:
            return
        self.service_handler.call_dispatch_mission()
        self.nextActionButton.setEnabled(True)
        self.stopButton.setEnabled(True)

    # Service to stop the actual action BT
    def stop_behavior_tree_service(self):
        if not self.spawned_marker:
            return
        self.service_handler.stop_behavior_tree()

    # Service to rotate the actual interactive marker
    def rotate_marker_service(self):
        if not self.spawned_marker:
            return
        response = self.service_handler.rotate_marker()
        self.view_on_marker(response.orientation)

    # Service to translate the actual interactive marker
    def move_marker_service(self, x, z):
        if not self.spawned_marker:
            return
        self.service_handler.move_marker(x, z)
        new_x = round(self.lcdXposition.value() + x, 2)
        new_z = round(self.lcdYposition.value() + z, 2)
        self.change_lcd_values(new_x, new_z)

    def shutdown(self):
        return
