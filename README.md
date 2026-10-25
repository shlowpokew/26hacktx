# ROS 2 robot stack

ROS 2 Jazzy (Ubuntu Noble) in Docker for a 64-bit ARM (Raspberry Pi) or x86
Linux robot: SLAM Toolbox, Nav2, robot_localization, Cyclone DDS, rosbag2 and
a UDP bridge to an Arduino UNO R4 WiFi. No ROS install is needed on the host.

**Set it up once, then just `git push`.** After the one-time install, the robot
checks GitHub every minute, pulls new commits, and rebuilds and restarts only
what changed. You never SSH in to update, rebuild, or restart anything.

```
your laptop                 GitHub                 robot (every ~1 min)
edit -> commit -> push  ->  main branch  ->  git pull -> rebuild what changed -> restart ROS
```

## One-time setup (on the robot)

1. Flash 64-bit Ubuntu Server or 64-bit Raspberry Pi OS and connect it to the network.
2. Give the robot read access to the repo. For a private repo, add a deploy key
   or your SSH key to GitHub. The key must have **no passphrase**, because
   updates run unattended.
3. Clone and install:

   ```sh
   git clone git@github.com:<you>/<repo>.git ~/etc/ros2
   cd ~/etc/ros2
   bash install.sh
   ```

The installer (it uses `sudo`):

- installs Docker if it's missing and enables it at boot
- builds the ROS image, pinning the base image digest in `.env`
- writes default SLAM parameters to `config/slam.yaml`
- starts the `mapping` service, which comes back after reboots and crashes
- installs `ros2-update.timer`, the auto-updater

Then **commit the files the installer generated** (`.env`, `config/slam.yaml`,
`packages.lock.txt`, `image-id.txt`) and push them. Updates can only
fast-forward, so the robot's checkout must not hold uncommitted edits that a
later commit would touch. A good habit: never edit files on the robot, only push.

That's the whole setup. You can now close the SSH session for good.

## Day to day: just push

Edit on your laptop, commit, and `git push` to the branch the robot tracks
(normally `main`). Within about a minute the robot picks it up:

| You changed | What the robot does automatically |
| --- | --- |
| `config/*.yaml` | restarts `mapping` so ROS reloads the parameters |
| `mapping.launch.py`, `arduino_bridge.py`, `Dockerfile` | rebuilds the image (cached, usually seconds) and replaces the container |
| `compose.yaml`, `.env` | recreates the container with the new settings |
| `README.md` or other files not used by the stack | nothing (the container keeps running) |
| `install.sh` | logs a reminder; host-level changes need `bash install.sh` once |
| Arduino sketches | nothing: firmware is flashed separately (see [Arduino](#arduino-r4-wifi)) |

`maps/` and `bags/` live on the host and survive every update. Mapping pauses
for a few seconds while the container is replaced.

If a build fails (for example a typo in the Dockerfile), the old container
keeps running. Push a fix and the next check applies it.

### Seeing what happened

From any machine that can SSH in:

```sh
journalctl -u ros2-update -n 50          # update history and errors
sudo docker compose logs -f --tail 100   # ROS output (run in ~/etc/ros2; Ctrl+C stops following)
sudo docker compose ps                   # is mapping running?
```

### Controlling the updater

```sh
bash update.sh                                    # apply now instead of waiting
bash update.sh --force                            # rebuild + restart even with no new commit
sudo systemctl disable --now ros2-update.timer   # pause auto-update
sudo systemctl enable --now ros2-update.timer    # resume
ROS2_AUTO_UPDATE=0 bash install.sh                # install without the updater
```

If you `sudo docker compose stop` the service, it stays stopped: updates still
pull and build, but nothing starts until you run `sudo docker compose up -d`.

### When updates stop applying

`journalctl -u ros2-update` will show one of these:

- **Cannot fast-forward**: someone edited or committed on the robot. Run
  `git status` in `~/etc/ros2`, then discard (`git checkout -- <file>`) or
  push the edits, and run `bash update.sh`.
- **Permission denied (publickey)**: the robot's key can't read the repo, or
  it has a passphrase. Fix the key (setup step 2).
- **no upstream**: run `git branch --set-upstream-to=origin/main`.

## Arduino R4 WiFi

The `mapping` service includes a UDP bridge that runs with or without an
Arduino attached. Set the Arduino's IP in `.env`, then commit and push. The
update recreates the container with the new value.

```dotenv
ARDUINO_IP=192.168.1.50
ARDUINO_PORT=4210
ARDUINO_LOCAL_PORT=4211
```

- Telemetry: every newline-delimited message from the Arduino is published as
  `std_msgs/String` on `/arduino/telemetry`.
- Commands: a string published on `/arduino/command` is sent as
  `{"type":"command","data":"..."}`. The sketch understands `motor stop`,
  `motor brake`, `motor forward <0-255>`, `motor reverse <0-255>`, and
  `drive <left -255..255> <right -255..255>`.
- A `geometry_msgs/Twist` on `/cmd_vel` is converted to left/right PWM using
  differential-drive kinematics. Set `DRIVE_WHEEL_BASE_M` and
  `DRIVE_MAX_LINEAR_MPS` in `.env` to match the robot. The output is open-loop
  PWM; encoders are not used for speed control yet.
- The bridge pings every 250 ms. Firmware stops both motors if pings stop for
  1.5 s, and the bridge sends `drive 0 0` if `/cmd_vel` is quiet for
  `CMD_VEL_TIMEOUT_SEC` (default 0.5 s).

Firmware is **not** flashed by the auto-updater, because flashing resets the
board and can spin the motors. Flash [arduino_r4_wifi/arduino_r4_wifi.ino](arduino_r4_wifi/arduino_r4_wifi.ino)
from the Arduino IDE after setting the WiFi credentials and the robot's LAN IP.
Keep real WiFi credentials out of Git. For the one-motor bench test over USB,
see [arduino/motor_test/README.md](arduino/motor_test/README.md).

Try it from the robot:

```sh
sudo docker compose exec mapping bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 topic echo /arduino/telemetry'
sudo docker compose exec mapping bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 topic pub --once /arduino/command std_msgs/msg/String "{data: motor stop}"'
```

The Arduino and robot must be on a network that lets UDP through on ports
4210 and 4211. Initial DRV8833 wiring uses D5/D6 for the left channel, D9/D10
for the right channel, and D8 for shared nSLEEP. Match both channels to the
motor driver's IN1/IN2 pins and verify wheel direction with the wheels raised
before putting the rover on the ground.

## Maps and bags

```sh
# Save the current map to maps/site.{yaml,pgm}
sudo docker compose exec mapping bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 run nav2_map_server map_saver_cli -f /maps/site'
# Record a session to bags/ (add your encoder/IMU topics)
sudo docker compose exec mapping bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 bag record -o /bags/session /scan /odom /tf /tf_static'
```

To replay a bag, set `USE_SIM_TIME=true` in `.env`, push, stop the live sensor
drivers, and play the bag with `--clock`. Set it back to `false` for live runs.

## What's still needed for real mapping

The container builds LDROBOT's ROS 2 driver v3.0.3 for the LD19/D300 and
launches it on `/dev/lidar` at 230400 baud. A scan filter publishes `/scan` in
the `laser` frame, retaining ranges from 0.15 m through 8.0 m and marking
other readings NaN. Encoder inputs default to BCM GPIO 17/18 on the left and
22/23 on the right. The odometry node publishes `/odom` and `odom -> base_link`
at 50 Hz.
The installer adds the persistent udev rule for USB ID `1e4e:aa68`; reconnect
the LiDAR after installation and confirm it appears as `/dev/lidar`. Set
`LIDAR_X_M`, `LIDAR_Y_M`, `LIDAR_Z_M`, and `LIDAR_YAW_RAD` in `.env` to match
its mounted pose. `LIDAR_MIN_RANGE_M` and `LIDAR_MAX_RANGE_M` configure the
scan window. The Compose service expects the LiDAR to be connected when it
starts. Set `GPIO_DEVICE` if the host exposes the Pi 5 GPIO chip under a
different `/dev/gpiochip*` name; the container maps the selected device to
`/dev/gpiochip4`.

Calibrate `DRIVE_WHEEL_RADIUS_M`, `DRIVE_WHEEL_BASE_M`, and
`ENCODER_TICKS_PER_REV` for the installed wheels and encoder gearing. Rotate a
wheel exactly once and measure the change in the corresponding GPIO Zero
encoder `steps` to determine ticks per wheel revolution. Set
`LEFT_ENCODER_SIGN` or `RIGHT_ENCODER_SIGN` to `-1` if forward wheel rotation
decreases that encoder's count.

SLAM Toolbox now receives wheel odometry on `/odom`; verify the encoder pin
numbers, tick count, wheel radius, and wheel spacing on the assembled rover.
The expected TF chain is `map -> odom -> base_link -> laser`. Set
`scan_topic`, `base_frame`, `odom_frame`, and `map_frame` in `config/slam.yaml`
to match the robot. Nav2 is installed but not launched.

The human-follow, breadcrumb return, onboard buttons/display, and SOS audio
nodes described by the full spec are not implemented yet. Nav2 is installed
but is not launched. Use the same `ROS_DOMAIN_ID` on every machine that talks
to the robot. Host networking is Linux-only, and Tailscale alone does not
carry DDS discovery across networks.

## Reproducibility

The base image is pinned by digest in `.env`. `packages.lock.txt` records the
apt package versions of the last install, but rebuilds don't enforce them. To
copy an exact build to another robot of the same architecture:

```sh
sudo docker save local/ros2-robot:jazzy | gzip > ros2-jazzy-image.tar.gz
# on the other robot, inside this repo:
gunzip -c ros2-jazzy-image.tar.gz | sudo docker load
sudo docker compose up -d --no-build
```

## Validation status

Written on macOS. Shell syntax is checked locally. Docker builds, the systemd
timer and hardware behavior have not been run on the robot yet.
