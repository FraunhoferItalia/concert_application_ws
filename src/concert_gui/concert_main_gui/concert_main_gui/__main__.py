#!/usr/bin/env python

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
sys.SELECT_QT_BINDING = "pyside"


from qt_gui.plugin import Plugin
from rqt_gui.main import Main

from python_qt_binding.QtWidgets import QWidget, QSplashScreen
from python_qt_binding.QtGui import QPixmap
from python_qt_binding.QtCore import QTimer

from python_qt_binding import loadUi
from ament_index_python import get_package_share_directory

from rqt_embed_window.embed_window_widget import EmbedWindowWidget
from rqt_rviz_transport_widget.rviz_transport_widget import RqtRvizTransportWidget
from rqt_rviz_drilling_widget.rviz_drilling_widget import RqtRvizDrillingWidget


from camera_widget.camera_widget import CameraWidget

from python_qt_binding import QtCore

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String, Float64, Bool, Int8
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import yaml
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
import time
from action_msgs.msg import GoalStatusArray

topic_subscriber = None


# Handle signal to update visual elements on the GUI
class SignalsHandler(QtCore.QObject):
    status_signal = QtCore.Signal(String)
    battery_signal = QtCore.Signal(Float64)
    emergency_signal = QtCore.Signal(Bool)
    bt_status_signal = QtCore.Signal(Int8)

    def __init__(self):
        super(SignalsHandler, self).__init__()

# Handle ROS subscribers
class SubscriberHandler(Node):
    def __init__(self):
        super().__init__("subscriber_handler")

        yaml_config_path = os.path.join(
            get_package_share_directory("concert_main_gui"),
            "config/yaml_files",
            "gui_config.yaml",
        )

        yaml_config_file = open(yaml_config_path, "r")
        self.yaml_conf = yaml.safe_load(yaml_config_file)

        self.subscription_status = self.create_subscription(
            String,
            self.yaml_conf["robot_status"]["topic_name"],
            self.status_callback,
            1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.subscription_status

        self.subscription_battery = self.create_subscription(
            Float64,
            self.yaml_conf["battery_status"]["topic_name"],
            self.battery_callback,
            10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.subscription_battery

        self.subscription_emergency = self.create_subscription(
            Bool,
            self.yaml_conf["emergency_status"]["topic_name"],
            self.emergency_callback,
            10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.subscription_emergency

        self.subscription_bt_status = self.create_subscription(
            GoalStatusArray,
            self.yaml_conf["bt_status"]["topic_name"],
            self.bt_status_callback,
            1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.subscription_status

        self.signals = SignalsHandler()

    # Get messages from mission status
    def status_callback(self, msg):
        try:
            self.signals.status_signal.emit(msg.data)
        except Exception as e:
            print(e)
        time.sleep(5)
        self.signals.status_signal.emit("No Messages")

    # Get battery status from mobile base
    def battery_callback(self, msg):
        try:
            self.signals.battery_signal.emit(msg.data)
        except Exception as e:
            print(e)

    # Get emergency status button of mobile base
    def emergency_callback(self, msg):
        try:
            self.signals.emergency_signal.emit(msg.data)
        except Exception as e:
            print(e)

    # Get status of the behavior tree
    def bt_status_callback(self, msg):
        goal_status_array = msg
        goal_status = goal_status_array
        goal_status = goal_status.status_list[-1].status
        try:
            self.signals.bt_status_signal.emit(goal_status)
        except Exception as e:
            print(e)


class MainGui(Plugin):
    def __init__(self, context):

        super(MainGui, self).__init__(context)
        self.setObjectName("MainGui")

        self.node = rclpy.create_node("concert_main_gui")

        rviz_gui_name = "rviz_file"
        rviz_gui__param_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING, description="Rviz GUI to load."
        )
        self.node.declare_parameter("rviz_file", "", rviz_gui__param_descriptor)

        rviz_file = ""
        if self.node.has_parameter(rviz_gui_name):
            rviz_file = self.node.get_parameter(rviz_gui_name).value
            self.node.get_logger().info(f"Loading the  {rviz_gui_name}: {rviz_file}")
        else:
            self.node.get_logger().error(
                f"Parameter {rviz_gui_name} does not exist. Exiting..."
            )
            return

        pixmap = QPixmap("src/0_concert_gui/concert_main_gui/config/images/concert_large.gif")
        self.splash = QSplashScreen(pixmap)
        self.splash.show()

        self._widget = QWidget()

        if rviz_file.endswith("transport"):
            ui_file = os.path.join(
                get_package_share_directory("concert_main_gui"),
                "config/gui_files",
                "Main_transport.ui",
            )

            loadUi(
                ui_file,
                self._widget,
                {"RqtRvizTransportWidget": RqtRvizTransportWidget, "EmbedWindowWidget": EmbedWindowWidget},
            )
        elif rviz_file.endswith("drilling"):
            ui_file = os.path.join(
                get_package_share_directory("concert_main_gui"),
                "config/gui_files",
                "Main_drilling.ui",
            )

            loadUi(
                ui_file,
                self._widget,
                {"RqtRvizDrillingWidget": RqtRvizDrillingWidget, "EmbedWindowWidget": EmbedWindowWidget},
            )
        else:
            self.node.get_logger().error("Rviz file param not recognized! The name must finish with 'drilling' or 'transport'")
            raise ValueError
        
        self._widget.setObjectName("MainUi")

        self._widget.cameraWidget.addWidget(CameraWidget())
        self._widget.behaviourTreeWidget.add_external_window_widget("groot2 --nosplash 1")

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number())
            )
        context.add_widget(self._widget)

        topic_subscriber.signals.status_signal.connect(self.statusCallback)
        topic_subscriber.signals.battery_signal.connect(self.batteryCallback)
        topic_subscriber.signals.emergency_signal.connect(self.emergencyCallback)
        topic_subscriber.signals.bt_status_signal.connect(self.behaviorTreeStatusCallback)

        self._widget.statusRuntimeLabel.setStyleSheet("color: red;")
        self._widget.statusRuntimeLabel.setText("No Messages")

        self._widget.btRuntimeLabel.setStyleSheet("color: red;")
        self._widget.btRuntimeLabel.setText("Inactive")

        self.closeSplash()

        return


    # Close splashscreen
    def closeSplash(self):
        self.splash.close()

    # Get messages from mission status
    def statusCallback(self, data):
        if data == "No Messages":
            self._widget.statusRuntimeLabel.setStyleSheet("color: red;")
        else:
            self._widget.statusRuntimeLabel.setStyleSheet("color: #179C7D;")
        self._widget.statusRuntimeLabel.setText(data)

    # Get emergency status button of mobile base and set the corresponding image
    def emergencyCallback(self, data):
        if data == True:
            self._widget.emergencyLabel.setPixmap(
                "src/0_concert_gui/concert_main_gui/config/images/redlight.png"
            )
        else:
            self._widget.emergencyLabel.setPixmap(
                "src/0_concert_gui/concert_main_gui/config/images/greenlight.png"
            )

    # Get battery status from mobile base and set image to show the battery status
    def batteryCallback(self, data):
        if data <= 100.0 and data >= 80.0:
            self._widget.batteryLabel.setPixmap(
                "src/0_concert_gui/concert_main_gui/config/images/discharging_100.png"
            )
        elif data < 80.0 and data >= 50.0:
            self._widget.batteryLabel.setPixmap(
                "src/0_concert_gui/concert_main_gui/config/images/discharging_70.png"
            )
        elif data < 50.0 and data >= 30.0:
            self._widget.batteryLabel.setPixmap(
                "src/0_concert_gui/concert_main_gui/config/images/discharging_40.png"
            )
        elif data < 30.0 and data >= 20.0:
            self._widget.batteryLabel.setPixmap(
                "src/0_concert_gui/concert_main_gui/config/images/discharging_10.png"
            )
        else:
            self._widget.batteryLabel.setPixmap(
                "src/0_concert_gui/concert_main_gui/config/images/discharging_0.png"
            )

    # Get status of the behavior tree and shows on the GUI
    def behaviorTreeStatusCallback(self, data):
        if data == 0:
            self._widget.btRuntimeLabel.setText("Unknown")
        elif data == 1:
            self._widget.btRuntimeLabel.setStyleSheet("color: #179C7D;")
            self._widget.btRuntimeLabel.setText("Accepted")
        elif data == 2:
            self._widget.btRuntimeLabel.setStyleSheet("color: #F6BE00;")
            self._widget.btRuntimeLabel.setText("Executing")
        elif data == 3:
            self._widget.btRuntimeLabel.setStyleSheet("color: orange;")
            self._widget.btRuntimeLabel.setText("Canceling")
        elif data == 4:
            self._widget.btRuntimeLabel.setStyleSheet("color: #179C7D;")
            self._widget.btRuntimeLabel.setText("Succeeded")
        elif data == 5:
            self._widget.btRuntimeLabel.setStyleSheet("color: red;")
            self._widget.btRuntimeLabel.setText("Canceled")
        elif data == 6:
            self._widget.btRuntimeLabel.setStyleSheet("color: red;")
            self._widget.btRuntimeLabel.setText("Aborted")

    def shutdown(self):
        self._widget.shutdown()
        exit()
        return

    def closeEvent(self, event):
        self.shutdown()
        return

    def shutdown_plugin(self):
        self.shutdown()
        return


def main(args=None):
    rclpy.init(args=args)

    global topic_subscriber

    topic_subscriber = SubscriberHandler()

    executor = MultiThreadedExecutor()
    executor.add_node(topic_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    sys.exit(Main().main(sys.argv, standalone="concert_main_gui.__main__"))


if __name__ == "__main__":
    main()
