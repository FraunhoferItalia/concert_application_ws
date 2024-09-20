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

from python_qt_binding import loadUi
import python_qt_binding.QtCore as QtCore
from python_qt_binding.QtGui import QImage, QPixmap, QIcon
from python_qt_binding.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget, QMessageBox

import cv2

from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from ament_index_python import get_package_share_directory
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import yaml
import datetime

class CameraSignals(QtCore.QObject):
    camera_front_signal = QtCore.Signal(CompressedImage)
    camera_eef_signal = QtCore.Signal(CompressedImage)


    def __init__(self):
        super(CameraSignals, self).__init__()


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__("camera_subscriber")

        gui_config_file = self.declare_parameter("gui_config", "gui_config").get_parameter_value().string_value

        yaml_config_path = os.path.join(
            get_package_share_directory("concert_main_gui"),
            "config/yaml_files",
            gui_config_file + ".yaml",
        )

        yaml_config_file = open(yaml_config_path, "r")
        self.yaml_conf = yaml.safe_load(yaml_config_file)

        self.camera_front_subscription = self.create_subscription(
            CompressedImage,
            self.yaml_conf["camera_front_viewer"]["topic_name"],
            self.camera_front_callback,
            10,
        )
        self.camera_front_subscription

        self.camera_eef_subscription = self.create_subscription(
            CompressedImage,
            self.yaml_conf["camera_eef_viewer"]["topic_name"],
            self.camera_eef_callback,
            10,
        )
        self.camera_eef_subscription

        self.signals = CameraSignals()

    # View front camera of the mobile base
    def camera_front_callback(self, msg):
        try:
            self.signals.camera_front_signal.emit(msg)
        except Exception as e:
            print(e)

    # View end effector camera
    def camera_eef_callback(self, msg):
        try:
            self.signals.camera_eef_signal.emit(msg)
        except Exception as e:
            print(e)


class CameraWidget(QWidget):
    def __init__(self, plugin=None):
        super(CameraWidget, self).__init__()
        self._plugin = plugin
        self.bridge = CvBridge()

        package_share_directory = get_package_share_directory("concert_main_gui")

        ui_file = os.path.join(
            package_share_directory,
            "config/gui_files",
            "CameraWidget.ui",
        )

        loadUi(ui_file, self)

        self.createSubscriber()

        self.camera_subscriber.signals.camera_front_signal.connect(self.cameraCallback)

        self.record_video = False

        self.changeCameraButton.setStyleSheet("background-color:#179C7D")
        self.takePictureButton.setStyleSheet("background-color:#179C7D")
        self.startRecordButton.setStyleSheet("background-color:#179C7D")
        self.stopRecordButton.setStyleSheet("background-color:#cc0000")

        changeCameraIcon = os.path.join(package_share_directory, "config/images", "change_camera.png")
        takePictureIcon = os.path.join(package_share_directory, "config/images", "camera.png")
        startRecordIcon = os.path.join(package_share_directory, "config/images", "recording.png")
        stopRecordIcon = os.path.join(package_share_directory, "config/images", "stoprecording.png")

        self.changeCameraButton.setIcon(QIcon(changeCameraIcon))
        self.takePictureButton.setIcon(QIcon(takePictureIcon))
        self.startRecordButton.setIcon(QIcon(startRecordIcon))
        self.stopRecordButton.setIcon(QIcon(stopRecordIcon))

        self.stopRecordButton.setEnabled(False)

        self.changeCameraButton.clicked.connect(self.changeCamera)
        self.takePictureButton.clicked.connect(self.takePicture)
        self.startRecordButton.clicked.connect(self.startRecordVideo)
        self.stopRecordButton.clicked.connect(self.stopRecordVideo)

        self.front_camera = True

        self.cv_image = None

    # Switch the camera
    def changeCamera(self):
        if self.front_camera == True:
            self.front_camera = False
            self.camera_subscriber.signals.camera_front_signal.disconnect(self.cameraCallback)
            self.camera_subscriber.signals.camera_eef_signal.connect(self.cameraCallback)
        else:
            self.front_camera = True
            self.camera_subscriber.signals.camera_eef_signal.disconnect(self.cameraCallback)
            self.camera_subscriber.signals.camera_front_signal.connect(self.cameraCallback)
    
    # Take a picture of the current view from camera
    def takePicture(self):
        cv2.cvtColor(self.cv_image, cv2.COLOR_RGB2BGR)
        cv2.imwrite('image'+str(datetime.datetime.now())+'.png', cv2.cvtColor(self.cv_image, cv2.COLOR_RGB2BGR))
        self.msg = QMessageBox()
        self.msg.setIcon(QMessageBox.Information)
        self.msg.setText("Picture Saved")
        self.msg.setWindowTitle("Info")
        self.msg.setStandardButtons(QMessageBox.Close)
        closeTimer = QtCore.QTimer(self.msg, singleShot=True, interval=1000, 
            timeout=self.msg.close)
        closeTimer.start()    
        self.msg.exec()

    # Start Video Recording
    def startRecordVideo(self):
        fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
        self.video = cv2.VideoWriter('video' + str(datetime.datetime.now()) + '.avi', fourcc,30, (self.cv_image.shape[1], self.cv_image.shape[0]))
        self.startRecordButton.setEnabled(False)
        self.stopRecordButton.setEnabled(True)
        self.record_video = True
        self.msg = QMessageBox()
        self.msg.setIcon(QMessageBox.Information)
        self.msg.setText("Start Recording")
        self.msg.setWindowTitle("Info")
        self.msg.setStandardButtons(QMessageBox.Close)
        closeTimer = QtCore.QTimer(self.msg, singleShot=True, interval=1000, 
            timeout=self.msg.close)
        closeTimer.start()    
        self.msg.exec()

    # Stop Video Recording
    def stopRecordVideo(self):
        self.video.release()
        self.startRecordButton.setEnabled(True)
        self.stopRecordButton.setEnabled(False)
        self.msg = QMessageBox()
        self.msg.setIcon(QMessageBox.Information)
        self.msg.setText("Stop Recording")
        self.msg.setWindowTitle("Info")
        self.msg.setStandardButtons(QMessageBox.Close)
        closeTimer = QtCore.QTimer(self.msg, singleShot=True, interval=1000, 
            timeout=self.msg.close)
        closeTimer.start()    
        self.msg.exec()

    def createSubscriber(self):
        self.camera_subscriber = CameraSubscriber()

        executor = MultiThreadedExecutor()
        executor.add_node(self.camera_subscriber)

        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

    # Get image stream from the camera
    def cameraCallback(self, data):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "rgb8")

        #cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")

        self.cv_image = cv_image

        number_of_channels = len(cv_image.shape)
        if number_of_channels == 3:
            height, width, channels = cv_image.shape
            if channels == 3:
                img_format = QImage.Format_RGB888
            if channels == 4:
                img_format = QImage.Format_ARGB32
        else:
            height, width = cv_image.shape
            channels = 1
            img_format = QImage.Format_Grayscale8
        bytes_per_line = channels * width
        q_img = QImage(cv_image, width, height, bytes_per_line, img_format)

        self.camera_label.setPixmap(
            QPixmap.fromImage(q_img).scaled(
                self.camera_label.width(),
                self.camera_label.height(),
                QtCore.Qt.KeepAspectRatio,
            )
        )

        if self.record_video == True:
            self.video.write(cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR))

        self.show()
        return
