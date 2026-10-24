# ROS 2 robot stack

ROS 2 Jazzy on Ubuntu Noble inside Docker, for 64-bit ARM (Pi) or x86 Linux.
Includes SLAM Toolbox, Nav2, robot_localization, Cyclone DDS, ROS bag recording,
and build tools. Host installer supports systemd Ubuntu/Debian, including
64-bit Raspberry Pi OS. No ROS installation is needed on the host.

## Install on the robot

Copy this entire directory to ~/etc/ros2 on the Linux robot, then run:

```sh
cd ~/etc/ros2
bash install.sh
```

The installer uses sudo, installs Docker if absent, enables Docker at boot,
builds the ROS image, copies upstream SLAM defaults into config/slam.yaml,
and starts mapping. Existing Docker installations are preserved; conflicting
packages may need resolving if Docker's package installer reports a conflict.
Tailscale should run separately on the host.

## Required hardware integration

This is a software foundation, not a finished robot configuration. No sensor
model was supplied, so no LiDAR driver or fabricated odometry is configured.
SLAM Toolbox here assumes a 2D LiDAR. It needs /scan plus the TF chain
odom -> base_link -> your_laser_frame with valid timestamps. Set scan_topic,
base_frame, odom_frame and map_frame in config/slam.yaml to match your robot.
Wheel encoders/IMU need their own drivers and a configured odometry estimator.
Nav2 is installed but not launched; motor control/navigation need robot-specific setup.

For a USB LiDAR, add its driver to the Dockerfile and a separate Compose
service with an explicit devices mapping using /dev/serial/by-id/... . Do not
use privileged mode or map all of /dev. Ethernet LiDAR usually needs a host
interface/IP setup instead. Set ROS_DOMAIN_ID consistently on participating
machines. Host networking here is intended for Linux. Tailscale alone does not
make ROS DDS multicast discovery work across remote networks.

## Operation

```sh
sudo docker compose ps
sudo docker compose logs -f --tail 100
sudo docker compose exec mapping bash
sudo docker compose restart
sudo docker compose stop       # stays stopped across reboots
sudo docker compose up -d      # enables running again
```

Docker's restart policy starts the existing container after a host reboot.
The launch wrapper shuts down if a launched ROS process exits, allowing the
container to restart. A hung node is not detected or recovered automatically.
No automatic host reboot or motor-control command is configured.

Maps and bags persist in maps/ and bags/. To save a map while mapping:

```sh
sudo docker compose exec mapping bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 run nav2_map_server map_saver_cli -f /maps/site'
```

Record inputs (add your actual raw encoder/IMU topics):

```sh
sudo docker compose exec mapping bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 bag record -o /bags/session /scan /odom /tf /tf_static'
```

For replay, stop live sensor drivers, set USE_SIM_TIME=true in .env, recreate
mapping with `sudo docker compose up -d`, and play the bag with --clock in a
container using the same domain and network. Restore false for live operation.

## Reproducibility

Keep Dockerfile, compose.yaml, mapping.launch.py, .env, config/slam.yaml,
packages.lock.txt, image-id.txt, and your driver source commits in version
control. No credentials belong here. The installer pins the base image digest
on first install; apt package versions can change on later clean builds.
packages.lock.txt records versions but does not enforce them during rebuilds.
For exact deployment of this build to another matching architecture, archive
and transfer the built image:

```sh
sudo docker save local/ros2-robot:jazzy | gzip > ros2-jazzy-image.tar.gz
sha256sum ros2-jazzy-image.tar.gz > ros2-jazzy-image.tar.gz.sha256
# On the other host, with Docker installed and this directory copied:
sha256sum -c ros2-jazzy-image.tar.gz.sha256
gunzip -c ros2-jazzy-image.tar.gz | sudo docker load
sudo systemctl enable --now docker
sudo docker compose up -d --no-build
```

Archive the calibration/config and a reference bag alongside that image.
A locked software environment does not guarantee identical SLAM outputs.

## Validation status

Prepared on macOS. Shell syntax, Python syntax and YAML parsing were
checked locally. Docker Compose validation could not run because the Mac lacks
the Compose plugin; the installer validates Compose on the robot. No Docker daemon or permitted SSH connection to the Pi was
available, so the image build, hardware mapping and boot startup remain
unverified on the robot.
