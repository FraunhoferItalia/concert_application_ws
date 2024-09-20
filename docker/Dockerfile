FROM fraunhoferitalia/rosbim:rosbim_release

ENV ROS_WS_DIR="/concert_application_ws"
WORKDIR ${ROS_WS_DIR}

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# for rviz_plugins
RUN apt-get -y update && apt-get -y install \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rviz-common \
    ros-${ROS_DISTRO}-rviz-default-plugins \
    ros-${ROS_DISTRO}-rviz-visual-tools \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-nav2-rviz-plugins

RUN apt-get -y update && apt-get -y install \    
    fuse libfuse2 \
    wmctrl

RUN pip install mathutils

# download thhttps://www.behaviortree.dev/groot/
RUN apt-get install -y wget
RUN wget -O groot2 https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.6.1-x86_64.AppImage -P /concert_application_ws
RUN chmod +x ./groot2
RUN mv "groot2" "../usr/bin/"

# Required by pickIK
RUN apt-get -y update && apt-get -y install \
    librange-v3-dev

# Visual studio configuration, update it and register aliases
COPY "/.vscode" "${ROS_WS_DIR}/.vscode"

RUN echo "source ${ROS_WS_DIR}/environment.sh" >> ~/.bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc && \
    echo "alias rbuild='(cd ${ROS_WS_DIR} && colcon build --cmake-args -DBUILD_TESTING=OFF && source install/setup.sh)'" >> ~/.bash_aliases && \
    echo "alias rclean='(cd ${ROS_WS_DIR} && rm -rf install/ log/ build/)'"  >> ~/.bash_aliases

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
