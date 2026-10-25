#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ $(uname -s) != Linux ]]; then
  echo 'Run this installer on the Linux robot, not macOS.' >&2; exit 1
fi
case "$(uname -m)" in aarch64|x86_64) ;; *) echo 'Requires 64-bit ARM or x86 Linux.' >&2; exit 1 ;; esac
if ! command -v systemctl >/dev/null; then
  echo 'Requires a systemd host.' >&2; exit 1
fi
source /etc/os-release
case "$ID" in
  ubuntu|debian) docker_os="$ID"; docker_suite="${VERSION_CODENAME:?}" ;;
  raspbian) echo 'Use 64-bit Raspberry Pi OS (Debian) or Ubuntu Server.' >&2; exit 1 ;;
  *) echo 'Installer supports Ubuntu and Debian hosts.' >&2; exit 1 ;;
esac
sudo -v
if ! command -v docker >/dev/null; then
  sudo apt-get update
  sudo apt-get install -y ca-certificates curl
  sudo install -m 0755 -d /etc/apt/keyrings
  sudo curl -fsSL "https://download.docker.com/linux/$docker_os/gpg" -o /etc/apt/keyrings/docker.asc
  sudo chmod a+r /etc/apt/keyrings/docker.asc
  printf 'deb [arch=%s signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/%s %s stable\n' \
    "$(dpkg --print-architecture)" "$docker_os" "$docker_suite" | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
  sudo apt-get update
  sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
fi
sudo docker compose version >/dev/null || {
  echo 'Docker exists but Compose plugin is missing. Install docker-compose-plugin from your Docker package repository.' >&2; exit 1;
}
sudo systemctl enable --now docker
sudo install -D -m 0644 udev/99-breadcrumbs-lidar.rules /etc/udev/rules.d/99-breadcrumbs-lidar.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
mkdir -p config maps bags
if [[ ! -f .env ]]; then
  sudo docker pull ros:jazzy-ros-base-noble
  base_digest=$(sudo docker image inspect ros:jazzy-ros-base-noble --format '{{index .RepoDigests 0}}')
  printf 'ROS_BASE=%s\nROS_DOMAIN_ID=42\nUSE_SIM_TIME=false\nARDUINO_IP=\nARDUINO_PORT=4210\nARDUINO_LOCAL_PORT=4211\nDRIVE_WHEEL_BASE_M=0.32\nDRIVE_WHEEL_RADIUS_M=0.05\nDRIVE_MAX_LINEAR_MPS=0.5\nDRIVE_MAX_ANGULAR_RPS=1.0\nCMD_VEL_TIMEOUT_SEC=0.5\nENCODER_FEEDBACK_TIMEOUT_SEC=0.3\nDRIVE_PID_KP=100.0\nDRIVE_PID_KI=40.0\nDRIVE_PID_KD=0.0\nDRIVE_PID_INTEGRAL_LIMIT=0.5\nLIDAR_SERIAL_PORT=/dev/lidar\nLIDAR_FRAME=laser\nLIDAR_X_M=0.0\nLIDAR_Y_M=0.0\nLIDAR_Z_M=0.0\nLIDAR_YAW_RAD=0.0\nLIDAR_MIN_RANGE_M=0.15\nLIDAR_MAX_RANGE_M=8.0\nGPIO_DEVICE=/dev/gpiochip4\nGPIO_CHIP=4\nLEFT_ENCODER_A_BCM=17\nLEFT_ENCODER_B_BCM=18\nRIGHT_ENCODER_A_BCM=22\nRIGHT_ENCODER_B_BCM=23\nENCODER_TICKS_PER_REV=240\nLEFT_ENCODER_SIGN=1\nRIGHT_ENCODER_SIGN=1\nODOM_PUBLISH_RATE_HZ=50\nODOM_FRAME=odom\nBASE_FRAME=base_link\nTRACKER_MIN_RANGE_M=0.10\nTRACKER_MAX_RANGE_M=3.0\nTRACKER_FRONT_ARC_DEG=60\nTRACKER_ASSOCIATION_DISTANCE_M=0.60\nTRACKER_TARGET_TIMEOUT_SEC=0.5\nHUMAN_DETECTOR_SCORE_THRESHOLD=0.937660\nHUMAN_DETECTOR_SCAN_NEAR_M=0.02\nHUMAN_DETECTOR_SCAN_FAR_M=10.0\nHUMAN_DETECTOR_THREADS=2\nSOS_NEAR_HUMAN_RANGE_M=3.0\nSOS_SEEK_STOP_DISTANCE_M=1.0\nSOS_SEEK_MAX_SPEED_MPS=0.12\nSOS_SEEK_MAX_ANGULAR_RPS=0.6\nLCD_I2C_BUS=1\nFOLLOW_DISTANCE_M=1.0\nFOLLOW_LINEAR_GAIN=0.7\nFOLLOW_LINEAR_DERIVATIVE_GAIN=0.05\nFOLLOW_ANGULAR_GAIN=1.5\nFOLLOW_ANGULAR_DERIVATIVE_GAIN=0.1\nOBSTACLE_STOP_DISTANCE_M=0.25\nOBSTACLE_STOP_ARC_DEG=25\nFOLLOW_DATA_TIMEOUT_SEC=0.5\nBACKTRACK_OBSTACLE_DISTANCE_M=0.60\nBACKTRACK_EMERGENCY_DISTANCE_M=0.30\nBACKTRACK_OBSTACLE_ARC_DEG=45\nBACKTRACK_DETOUR_SPEED_MPS=0.08\nBACKTRACK_ROBOT_RADIUS_M=0.22\nBACKTRACK_PLANNER_HORIZON_SEC=1.0\n' "$base_digest" > .env
fi
sudo docker compose build
if [[ ! -f config/slam.yaml ]]; then
  sudo docker compose run --rm --no-deps --entrypoint cat mapping \
    /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml > config/slam.yaml
fi
sudo docker compose config --quiet
# Keep a package inventory and exact image ID for this build.
sudo docker compose run --rm --no-deps --entrypoint cat mapping /opt/ros/package-manifest.txt > packages.lock.txt
sudo docker image inspect local/ros2-robot:jazzy --format '{{.Id}}' > image-id.txt
sudo docker compose up -d --no-build
# Auto-update: pull pushed commits every minute and apply them (see update.sh).
if [[ ${ROS2_AUTO_UPDATE:-1} == 1 ]]; then
  git rev-parse --verify --quiet '@{u}' >/dev/null ||
    echo 'Warning: this branch has no Git upstream, so auto-update has nothing to pull.' >&2
  sudo tee /etc/systemd/system/ros2-update.service >/dev/null <<EOF
[Unit]
Description=Pull ROS 2 robot repo from Git and apply changes
Wants=network-online.target
After=network-online.target docker.service

[Service]
Type=oneshot
ExecStart=/bin/bash "$PWD/update.sh"
EOF
  sudo tee /etc/systemd/system/ros2-update.timer >/dev/null <<'EOF'
[Unit]
Description=Check Git for ROS 2 robot updates every minute

[Timer]
OnActiveSec=30s
OnUnitInactiveSec=1min

[Install]
WantedBy=timers.target
EOF
  sudo systemctl daemon-reload
  sudo systemctl enable --now ros2-update.timer
  printf '\nAuto-update on: pushed commits are applied within about a minute.\n'
fi
printf '\nInstalled. Docker starts at boot; mapping restarts unless explicitly stopped.\n'
printf 'Mapping needs wheel odometry publishing /odom and the odom -> base_link TF.\n'
