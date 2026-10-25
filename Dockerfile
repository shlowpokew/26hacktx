ARG ROS_BASE=ros:jazzy-ros-base-noble
FROM ${ROS_BASE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-robot-localization \
    ros-jazzy-robot-state-publisher ros-jazzy-xacro ros-jazzy-tf2-tools \
    ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap \
    python3-colcon-common-extensions python3-rosdep python3-gpiozero python3-lgpio \
    build-essential git \
    && dpkg-query -W > /opt/ros/package-manifest.txt \
    && rm -rf /var/lib/apt/lists/*

# LDROBOT's ROS 2 driver supports the LD19, which is the D300 sensor in the spec.
RUN mkdir -p /opt/robot_ws/src \
    && git clone --depth 1 --branch v3.0.3 \
       https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git \
       /opt/robot_ws/src/ldlidar_stl_ros2 \
    && source /opt/ros/jazzy/setup.bash \
    && cd /opt/robot_ws \
    && colcon build --merge-install --packages-select ldlidar_stl_ros2 \
    && dpkg-query -W > /opt/ros/package-manifest.txt

COPY mapping.launch.py /opt/robot/mapping.launch.py
COPY arduino_bridge.py /opt/robot/arduino_bridge.py
COPY lidar_scan_filter.py /opt/robot/lidar_scan_filter.py
COPY encoder_odometry.py /opt/robot/encoder_odometry.py
COPY human_tracker.py /opt/robot/human_tracker.py
COPY rover_controller.py /opt/robot/rover_controller.py
COPY button_io.py /opt/robot/button_io.py
COPY mission_manager.py /opt/robot/mission_manager.py
COPY breadcrumb_manager.py /opt/robot/breadcrumb_manager.py
COPY backtrack_navigator.py /opt/robot/backtrack_navigator.py
RUN chmod +x /opt/robot/arduino_bridge.py /opt/robot/lidar_scan_filter.py /opt/robot/encoder_odometry.py \
    /opt/robot/human_tracker.py /opt/robot/rover_controller.py /opt/robot/button_io.py \
    /opt/robot/mission_manager.py /opt/robot/breadcrumb_manager.py /opt/robot/backtrack_navigator.py
CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && source /opt/robot_ws/install/setup.bash && exec ros2 launch /opt/robot/mapping.launch.py"]
