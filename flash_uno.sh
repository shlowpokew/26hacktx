#!/usr/bin/env bash
# Build the Arduino Uno firmware in Docker and flash it.
# Usage: bash flash_uno.sh [cargo flags]   e.g. --no-default-features (no motors)
#
# The ROS container holds the Uno's serial port, so it is stopped while
# flashing and started again afterwards, even if flashing fails. Flashing
# resets the Uno: keep the wheels off the ground.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

port=${UNO_PORT:-/dev/uno}
[[ -e $port ]] || {
  echo "$port not found. Plug in the Uno and run 'bash install.sh' once for the udev rule," >&2
  echo "or point UNO_PORT at it, e.g. UNO_PORT=/dev/ttyACM0 bash flash_uno.sh" >&2
  exit 1
}

sudo docker build -t local/uno-firmware --build-arg CARGO_ARGS="$*" arduino_uno_rs

was_running=$(sudo docker compose ps -q --status running mapping 2>/dev/null || true)
if [[ -n $was_running ]]; then
  echo 'Stopping mapping so it releases the Uno...'
  sudo docker compose stop mapping
  trap 'echo "Starting mapping again..."; sudo docker compose start mapping' EXIT
fi

sudo docker run --rm --device "$port:/dev/uno" local/uno-firmware
echo 'Uno flashed.'
