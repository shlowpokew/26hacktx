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
mkdir -p config maps bags
if [[ ! -f .env ]]; then
  sudo docker pull ros:jazzy-ros-base-noble
  base_digest=$(sudo docker image inspect ros:jazzy-ros-base-noble --format '{{index .RepoDigests 0}}')
  printf 'ROS_BASE=%s\nROS_DOMAIN_ID=42\nUSE_SIM_TIME=false\n' "$base_digest" > .env
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
printf '\nInstalled. Docker starts at boot; mapping restarts unless explicitly stopped.\n'
printf 'Mapping still needs a LiDAR driver publishing /scan and robot odometry/transforms.\n'
