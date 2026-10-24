ARG ROS_BASE=ros:jazzy-ros-base-noble
FROM ${ROS_BASE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-robot-localization \
    ros-jazzy-robot-state-publisher ros-jazzy-xacro ros-jazzy-tf2-tools \
    ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap \
    python3-colcon-common-extensions python3-rosdep build-essential git \
    && dpkg-query -W > /opt/ros/package-manifest.txt \
    && rm -rf /var/lib/apt/lists/*
COPY mapping.launch.py /opt/robot/mapping.launch.py
COPY arduino_bridge.py /opt/robot/arduino_bridge.py
RUN chmod +x /opt/robot/arduino_bridge.py
CMD ["ros2", "launch", "/opt/robot/mapping.launch.py"]
