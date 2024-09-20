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
import rospkg
import rviz
import subprocess

from qt_gui.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget


from .rviz_drilling_widget import RqtRvizDrillingWidget

#     Class encapsulating the GUI
class ConcertRvizDrillingGui(Plugin):
    
    def __init__(self, context):
        super(ConcertRvizDrillingGui, self).__init__(context)
        self.setObjectName("RqtRvizDrillingGui")

        # Create graphical user interface (GUI)
        self._widget = RqtRvizDrillingWidget(self)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number())
            )
        context.add_widget(self._widget)
        return

    def shutdown_plugin(self):
        self._widget.shutdown()
        return
