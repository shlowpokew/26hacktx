# ROS 2 robot stack

ROS 2 Jazzy on Ubuntu Noble inside Docker, for 64-bit ARM (Pi) or x86 Linux.
Includes SLAM Toolbox, Nav2, robot_localization, Cyclone DDS, ROS bag recording,
and build tools. Host installer supports systemd Ubuntu/Debian, including
64-bit Raspberry Pi OS. No ROS installation is needed on the host.

Keep the ROS service running on the robot and iterate from your GitHub repo:
pull changes on the robot, apply them to the running service, and keep going.
Docker runs in the background and restarts the service after reboots. You do
not need to manually delete containers or rerun the installer for each update.
Updates are applied when you run the commands below; there is no automatic
Git polling or live reload of running ROS nodes.

## Install on the robot

Clone your GitHub repository onto the Linux robot, keeping this directory at
~/etc/ros2 (or copy an existing Git checkout there), then run:

```sh
cd ~/etc/ros2
bash install.sh
```

The installer uses sudo, installs Docker if absent, enables Docker at boot,
builds the ROS image, copies upstream SLAM defaults into config/slam.yaml,
and starts mapping. Existing Docker installations are preserved; conflicting
packages may need resolving if Docker's package installer reports a conflict.
Tailscale should run separately on the host.

## Iterate with Git while the ROS service stays active

Make and commit changes in your development checkout, then push them to GitHub.
On the robot, pull those changes into the checkout used by Compose:

```sh
cd ~/etc/ros2
git pull --ff-only
```

Keep robot-specific edits committed or otherwise resolved before pulling.
Run Git on the host; the container does not need its own Git checkout.

For changes to config/slam.yaml, restart mapping to load the updated parameters:

```sh
sudo docker compose restart mapping
```

The config/ directory is mounted into the active container, so pulled config
changes are already visible inside it. Restarting makes ROS load them. The same
container is reused, with a brief interruption while mapping restarts.

For changes to mapping.launch.py, the Dockerfile, or installed dependencies:

```sh
sudo docker compose up -d --build mapping
```

Compose builds the updated image while the existing container runs, then
replaces the container if its image or service configuration changed. Mapping
briefly stops during replacement. The service continues in the background;
maps/ and bags/ persist on the host. You do not need to run `docker compose down`
first. Changes to compose.yaml or .env also require `docker compose up -d` to
apply the new service configuration.

Check the result after updating:

```sh
sudo docker compose ps
sudo docker compose logs -f --tail 100 mapping
```

Press Ctrl+C to stop following logs; the container keeps running.

Currently only config/, maps/, and bags/ are mounted. Source code such as
mapping.launch.py is copied into the image, so a Git pull alone does not update
that code inside the active container. If you add a ROS workspace for frequent
package development, mount it into the container, build it with
`colcon build --symlink-install`, source its `install/setup.bash`, and restart
the affected nodes after changes. C++ changes require rebuilding; running nodes
do not automatically reload edited code.

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
