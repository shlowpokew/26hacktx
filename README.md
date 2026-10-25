# ROS 2 robot stack

ROS 2 Jazzy (Ubuntu Noble) running in Docker on a 64-bit ARM (Raspberry Pi) or
x86 Linux robot. It has SLAM Toolbox, Nav2, robot_localization, Cyclone DDS,
rosbag2, and a USB serial bridge to an Arduino Uno. You don't need ROS
installed on the host.

You set the robot up once. After that it checks GitHub every minute, pulls new
commits, and rebuilds and restarts whatever changed, so updating it is just
`git push`. You shouldn't need to SSH in to update, rebuild or restart.

```
your laptop                 GitHub                 robot (every ~1 min)
edit -> commit -> push  ->  main branch  ->  git pull -> rebuild what changed -> restart ROS
```

## One-time setup (on the robot)

1. Flash 64-bit Ubuntu Server or 64-bit Raspberry Pi OS and get it on the network.
2. Give the robot read access to the repo. For a private repo, add a deploy key
   or your SSH key to GitHub. The key can't have a passphrase, since updates
   run with nobody around to type it.
3. Clone and install:

   ```sh
   git clone git@github.com:<you>/<repo>.git ~/etc/ros2
   cd ~/etc/ros2
   bash install.sh
   ```

The installer uses `sudo`. It installs Docker if it's missing and enables it
at boot, builds the ROS image (pinning the base image digest in `.env`),
writes default SLAM parameters to `config/slam.yaml`, starts the `mapping`
service, and installs `ros2-update.timer`, which is the auto-updater. The
`mapping` service comes back on its own after reboots and crashes.

When it finishes, commit the files it generated (`.env`, `config/slam.yaml`,
`packages.lock.txt`, `image-id.txt`) and push them. Updates only
fast-forward, so if the robot's checkout has uncommitted edits that a later
commit touches, updates stop. Easiest rule: don't edit files on the robot,
just push.

After that you can log out of the robot.

## Day to day

Edit on your laptop, commit, and `git push` to the branch the robot tracks
(normally `main`). The robot picks it up within about a minute:

| You changed | What the robot does |
| --- | --- |
| `config/*.yaml` | restarts `mapping` so ROS reloads the parameters |
| `*.py`, `mapping.launch.py`, `Dockerfile` | rebuilds the image (cached, usually seconds) and replaces the container |
| `compose.yaml`, `.env` | recreates the container with the new settings |
| `README.md` or other files the stack doesn't use | nothing, the container keeps running |
| `install.sh` | logs a reminder; host-level changes need `bash install.sh` once |
| `arduino_uno_rs/` | logs a reminder to run `bash flash_uno.sh` (see [Arduino Uno](#arduino-uno)) |

`maps/` and `bags/` live on the host, so they survive every update. Mapping
pauses for a few seconds while the container is replaced.

If a build fails (a typo in the Dockerfile, say), the old container keeps
running. Push a fix and the next check picks it up.

### How an update works

The Pi doesn't reboot. Only the Docker container gets swapped. Every minute
`ros2-update.timer` runs `update.sh`, which:

1. pulls the new commit (`git pull --ff-only`);
2. builds a new image with `docker compose up -d --build`. Docker reuses
   cached layers, so changing a `.py` file rebuilds in seconds;
3. replaces the container, but only if the image, `compose.yaml` or `.env`
   changed. Docker sends the old container a stop signal, gives ROS up to 30 s
   to shut down, deletes it, and starts a new one from the new image. On
   startup the bridge sends the Uno a stop, and the motors stay off until ROS
   tells them to move;
4. or, if only `config/` changed, just restarts the existing container.

The container is set to `restart: unless-stopped` and Docker starts at boot,
so the stack comes back by itself after a crash or a reboot.

### Commands

Run these on the robot, in `~/etc/ros2`:

| What | Command |
| --- | --- |
| Apply the latest push now | `bash update.sh` |
| Rebuild and restart even with no new commit | `bash update.sh --force` |
| Is it running? | `sudo docker compose ps` |
| Watch the ROS logs | `sudo docker compose logs -f --tail 100` (Ctrl+C stops watching) |
| Restart without rebuilding | `sudo docker compose restart mapping` |
| Stop it (stays stopped across updates) | `sudo docker compose stop` |
| Start it again | `sudo docker compose up -d` |
| Throw the container away and make a new one | `sudo docker compose up -d --force-recreate` |
| Full clean rebuild, no cache | `sudo docker compose build --no-cache && sudo docker compose up -d` |
| Open a shell inside it | `sudo docker compose exec mapping bash` |
| Flash the Uno | `bash flash_uno.sh` |
| Update history and errors | `journalctl -u ros2-update -n 50` |
| Pause / resume auto-update | `sudo systemctl disable --now ros2-update.timer` / `enable --now` |
| Install without the auto-updater | `ROS2_AUTO_UPDATE=0 bash install.sh` |

A stopped service stays stopped. Updates still pull and build, but nothing
starts until you run `sudo docker compose up -d`.

### When updates stop applying

Check `journalctl -u ros2-update`. You'll usually see one of these:

- "Cannot fast-forward" means someone edited or committed on the robot. Run
  `git status` in `~/etc/ros2`, then either discard the edits
  (`git checkout -- <file>`) or push them, and run `bash update.sh`.
- "Permission denied (publickey)" means the robot's key can't read the repo,
  or it has a passphrase. Fix the key (setup step 2).
- "no upstream": run `git branch --set-upstream-to=origin/main`.

## Wiring and pin layout

```
             USB                        USB
 LD19 LiDAR ─────► Raspberry Pi 5 ◄──────────► Arduino Uno ──D5,D6,D9,D10,D8──► DRV8833 ──► motors
                    │ GPIO 17/18/22/23 ◄── motor encoders          │ A4/A5 (I2C)        ▲
                    │ GPIO 5/6        ◄── buttons A/B              ▼                    │
                                                                 16x2 LCD     6×AA battery pack
```

The Uno gets its power from the Pi over USB. The battery only powers the
motors.

### Arduino Uno pins

Both headers in board order, USB socket on the left:

```
Top header (digital)
  SCL  SDA  AREF GND  13   12   11   10   9    8   │  7    6    5    4    3    2    1    0
  LCD  LCD   ·    ·    ·    ·    ·   IN4  IN3  EEP │  ·   IN2  IN1   ·    ·    ·   (USB serial,
  SCL  SDA                           ───── DRV8833 ─────────────────            leave free)

Bottom header (power and analog)
  ·  IOREF RESET 3.3V  5V   GND  GND  VIN │  A0   A1   A2   A3   A4   A5
  ·    ·     ·    ·   LCD  LCD  DRV   ·   │   ·    ·    ·    ·   LCD  LCD
                      VCC  GND  GND                              SDA  SCL
```

| Uno pin | Goes to | Notes |
| --- | --- | --- |
| A4 (or the SDA pin by AREF) | LCD SDA | same signal, use either one |
| A5 (or the SCL pin by AREF) | LCD SCL | same signal, use either one |
| 5V | LCD VCC | |
| GND | LCD GND | |
| D5 | DRV8833 IN1 | left motor |
| D6 | DRV8833 IN2 | left motor |
| D9 | DRV8833 IN3 | right motor |
| D10 | DRV8833 IN4 | right motor |
| D8 | DRV8833 EEP (sleep) | the firmware holds it high, which turns the driver on |
| GND | DRV8833 GND | grounds have to be shared or nothing works |
| USB | Raspberry Pi USB | power and the serial link |
| D0, D1 | nothing | the USB serial link uses them |

### DRV8833 motor driver

| DRV8833 pin | Goes to |
| --- | --- |
| IN1, IN2, IN3, IN4 | Uno D5, D6, D9, D10 |
| EEP | Uno D8 |
| ULT | nothing (it's the fault output) |
| OUT1, OUT2 | left motor's two power wires |
| OUT3, OUT4 | right motor's two power wires |
| VCC | battery + |
| GND | battery − and Uno GND |

Some boards label the pins AIN1/AIN2/BIN1/BIN2 and AOUT1/AOUT2/BOUT1/BOUT2
instead, in the same order. Don't connect the battery to the Uno.

The DRV8833's [motor supply range tops out at 10.8 V](https://www.ti.com/lit/ds/symlink/drv8833.pdf).
6×AA is 9 V nominal and about 9.6 V fresh, so that's fine. The 25GA370 gear
motors are rated for 12 V and run slower at 9 V, so you may need to lower
`DRIVE_MAX_LINEAR_MPS` to what the wheels can actually reach.

If a wheel spins backwards, swap that motor's two wires on OUT1/OUT2 (or
OUT3/OUT4). Check this with the wheels off the ground before you put the
rover on the floor.

### Raspberry Pi GPIO

Each motor has six wires. The two power wires go to the DRV8833 and the four
encoder wires (VCC, GND, A, B) go to the Pi. The pin numbers below are
physical header positions, and "GPIO n" is the BCM number you put in `.env`.

```
                         3.3V  (1)  (2)  5V      ← encoder VCC (both motors)
     [LCD SDA, if on Pi] GPIO2 (3)  (4)  5V
     [LCD SCL, if on Pi] GPIO3 (5)  (6)  GND     ← encoder GND
                         GPIO4 (7)  (8)  GPIO14
                           GND (9)  (10) GPIO15
       left encoder A   GPIO17 (11) (12) GPIO18  left encoder B
                        GPIO27 (13) (14) GND
       right encoder A  GPIO22 (15) (16) GPIO23  right encoder B
                          3.3V (17) (18) GPIO24
                        GPIO10 (19) (20) GND
                         GPIO9 (21) (22) GPIO25
                        GPIO11 (23) (24) GPIO8
                           GND (25) (26) GPIO7
                         GPIO0 (27) (28) GPIO1
       button A          GPIO5 (29) (30) GND     ← other leg of both buttons
       button B          GPIO6 (31) (32) GPIO12
                        GPIO13 (33) (34) GND
                        GPIO19 (35) (36) GPIO16
                        GPIO26 (37) (38) GPIO20
                           GND (39) (40) GPIO21
```

| Pi pin | GPIO (BCM) | Goes to | `.env` setting |
| --- | --- | --- | --- |
| 11 | 17 | left encoder A | `LEFT_ENCODER_A_BCM` |
| 12 | 18 | left encoder B | `LEFT_ENCODER_B_BCM` |
| 15 | 22 | right encoder A | `RIGHT_ENCODER_A_BCM` |
| 16 | 23 | right encoder B | `RIGHT_ENCODER_B_BCM` |
| 2 or 4 | 5V | encoder VCC | |
| 6 (or any GND) | GND | encoder GND | |
| 29 | 5 | button A, other leg to GND | `BUTTON_A_BCM` |
| 31 | 6 | button B, other leg to GND | `BUTTON_B_BCM` |
| 3, 5 | 2, 3 | LCD SDA, SCL, only if the LCD is on the Pi instead of the Uno | |

The buttons use the Pi's internal pull-ups. Each button just shorts its GPIO
to GND when pressed, so you don't need a resistor.

Watch out with the encoders: their signals are 5 V, and the Pi's GPIO is
3.3 V and not 5 V tolerant. Put a level shifter or a resistor divider on each
A/B wire, or confirm the encoder outputs stay at or below 3.3 V, before you
connect them.

The LiDAR and the Uno can go in any of the Pi's USB ports. udev names them
`/dev/lidar` and `/dev/uno` no matter which port they're in.

## Arduino Uno

The Pi drives the motors through an Arduino Uno over USB. The Uno runs the
Rust firmware in [arduino_uno_rs/](arduino_uno_rs/README.md). Besides driving
the motors, it shows kaomoji on a 16x2 I2C LCD and refuses to drive into
anything the LiDAR sees closer than about 20 cm. The link is plain USB serial.

```
/cmd_vel ─► arduino_bridge.py (PID) ─USB, every 50 ms─► Uno ─► DRV8833 ─► motors
/scan ─────┘  /mission/state ─┘                          └──► LCD  \(^ω^)/
```

First-time setup, on the robot, with the Uno plugged in and the wheels off
the ground:

1. Run `bash install.sh` again so it installs the udev rule, then check that
   `ls -l /dev/uno` shows the Uno.
2. Flash the firmware: `bash flash_uno.sh`.
3. Set `UNO_DEVICE=/dev/uno` in `.env`, commit, and push.

Before step 3, the container starts without the Uno; the bridge logs that it
can't find the board and nothing drives. After step 3, the Uno has to be
plugged in when the container starts, same as the LiDAR.

### Flashing

```sh
bash flash_uno.sh                         # build and flash, motors enabled
bash flash_uno.sh --no-default-features   # dry run: reports PWM, never drives the motors
UNO_PORT=/dev/ttyACM0 bash flash_uno.sh   # before the udev rule is installed
```

`flash_uno.sh` builds the firmware inside a Docker image
(`arduino_uno_rs/Dockerfile`), so you don't need Rust or AVR tools on the Pi.
The first build downloads the toolchain and takes a few minutes; after that
it's cached. Since the ROS container holds the Uno's serial port, the script
stops `mapping`, flashes with avrdude, and starts `mapping` again. It
restarts `mapping` even if flashing fails.

The auto-updater never flashes the Uno, because flashing resets the board and
can spin the motors. When a push changes `arduino_uno_rs/`, the update log
reminds you to run `bash flash_uno.sh`.

### What the bridge does

`arduino_bridge.py` turns a `geometry_msgs/Twist` on `/cmd_vel` into left and
right wheel speed targets using differential-drive kinematics. Encoder
feedback from `/wheel/velocity` closes a PID loop on each wheel, and the
resulting signed PWM goes to the Uno. Tune `DRIVE_PID_KP`, `DRIVE_PID_KI`,
`DRIVE_PID_KD` and `DRIVE_PID_INTEGRAL_LIMIT` after you've calibrated wheel
radius and speed.

It stops the motors when:

- `/cmd_vel` has been quiet for `CMD_VEL_TIMEOUT_SEC` (default 0.5 s);
- encoder feedback is missing for `ENCODER_FEEDBACK_TIMEOUT_SEC`;
- the LiDAR scan goes stale while moving.

It publishes the reason on `/safety/warning`. During an active mission, a
stale `/cmd_vel` or `/scan` gets reported even if a controller already
commanded a stop.

Each frame it sends also carries the mission state and the nearest LiDAR
return in each 45° sector. If frames stop arriving for 0.5 s, the Uno stops on
its own. Status lines from the Uno show up on `/arduino/telemetry`, for
example `st=driving mission=follow l=150 r=130 ahead=212 near=88 bad=0`.

For bench tests, publish a string on `/arduino/command` to drive the wheels
directly for `MANUAL_COMMAND_SEC` (default 2 s). It accepts
`motor forward <0-255>`, `motor reverse <0-255>`,
`drive <left -255..255> <right -255..255>` and `motor stop`. Any `/cmd_vel`
message cancels it, and the Uno's LiDAR stop still applies.

To try it from the robot (wheels off the ground):

```sh
sudo docker compose exec mapping bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 topic echo /arduino/telemetry'
sudo docker compose exec mapping bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 topic pub --once /arduino/command std_msgs/msg/String "{data: motor forward 120}"'
```

## Maps and bags

```sh
# Save the current map to maps/site.{yaml,pgm}
sudo docker compose exec mapping bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 run nav2_map_server map_saver_cli -f /maps/site'
# Record a session to bags/ (add your encoder/IMU topics)
sudo docker compose exec mapping bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 bag record -o /bags/session /scan /odom /tf /tf_static'
```

To replay a bag, set `USE_SIM_TIME=true` in `.env`, push, stop the live
sensor drivers, and play the bag with `--clock`. Set it back to `false` for
live runs.

## LiDAR, odometry, follow, and return

### LiDAR

The container builds LDROBOT's ROS 2 driver v3.0.3 for the LD19/D300 and runs
it on `/dev/lidar` at 230400 baud. A filter node republishes the scan on
`/scan` in the `laser` frame, keeping ranges from 0.15 m to 8.0 m and setting
everything else to NaN. `LIDAR_MIN_RANGE_M` and `LIDAR_MAX_RANGE_M` change
that window.

The installer adds a udev rule for USB ID `1e4e:aa68`. Reconnect the LiDAR
after installing and check that it shows up as `/dev/lidar`. Set `LIDAR_X_M`,
`LIDAR_Y_M`, `LIDAR_Z_M` and `LIDAR_YAW_RAD` in `.env` to match how it's
mounted. The container won't start unless the LiDAR is plugged in.

### Odometry

Encoder inputs default to BCM GPIO 17/18 on the left and 22/23 on the right
(see [Raspberry Pi GPIO](#raspberry-pi-gpio)). The odometry node publishes
`/odom` and the `odom -> base_link` transform at 50 Hz. If the Pi 5 exposes
its GPIO chip under a different `/dev/gpiochip*` name, set `GPIO_DEVICE`; the
container maps whatever you pick to `/dev/gpiochip4`.

The 25GA370 motor listing says its quadrature encoder gives 240 pulses per
output-shaft revolution, so `ENCODER_TICKS_PER_REV` defaults to 240. To check,
turn a wheel exactly once and see how much that encoder's GPIO Zero `steps`
changes. Calibrate `DRIVE_WHEEL_RADIUS_M` and `DRIVE_WHEEL_BASE_M` on the
assembled rover. If forward rotation makes an encoder count go down, set
`LEFT_ENCODER_SIGN` or `RIGHT_ENCODER_SIGN` to `-1`. The listing also says the
encoders need a 5 V supply, which is why their A/B outputs need level
shifting before they reach the Pi (see the warning above).

SLAM Toolbox gets wheel odometry on `/odom`, so double-check the encoder pins,
tick count, wheel radius and wheel spacing on the real rover. The TF chain
should be `map -> odom -> base_link -> laser`. The installer sets
`scan_topic`, `base_frame`, `odom_frame` and `map_frame` in
`config/slam.yaml` to `/scan`, `base_link`, `odom` and `map`. If you change
`BASE_FRAME`, `ODOM_FRAME` or `MAP_FRAME` in `.env`, change them there too.

### Following a person

Following uses the [UPO LFE-PPN 2D-LiDAR detector](https://github.com/robotics-upo/upo_laser_people_detector)
and its pretrained ONNX model, which Docker downloads while building the
image. It runs on the Pi 5 with the ARM64 ONNX Runtime CPU provider, and only
while follow mode is on. The ROS node adapts the project's LFE-PPN input
normalization, anchor decoding, score threshold and NMS to this rover's scan
and target topics.

It picks the nearest detection in a ±60° arc in front and tracks it from scan
to scan to keep a lock on the same person. The rover drives to a 1.0 m
stand-off. It stops if scan or target data goes stale, or if there's an
obstacle within 0.25 m in the forward ±25° arc. The distance and heading
controllers both have derivative terms to cut down on overshoot.

### Buttons, breadcrumbs, and return

Button A starts follow-and-record from idle. Button B starts reverse
breadcrumb navigation while following. Hold B for 3 seconds to enter SOS, and
press A to go back to idle. The buttons default to BCM 5 and 6.

Breadcrumbs are saved to `/maps/breadcrumbs.json` every 0.25 m or 15° of
heading change. Each one has the map-frame position, heading and a Unix
timestamp, and the file is atomically replaced and flushed to disk on every
update. On the way back, the rover replays them from the latest pose toward
the start.

During return, each fresh LiDAR scan checks both the forward safety sector and
the rear travel corridor across ±45°. When something is closer than 0.60 m, a
local dynamic-window planner predicts short reverse-drive trajectories
against the scan points and picks a collision-free one that still makes
progress toward the current breadcrumb. Once the way is clear it goes back to
tracking that same waypoint. It stops if no candidate path is safe, an
obstacle is within 0.30 m, or scans go stale for 0.5 s.

### Display

The Grove RGB LCD connects to the Pi's I2C bus (default `/dev/i2c-1`), and the
container keeps retrying if it can't find it. The 16x2 display uses custom
5x8 glyphs to draw procedural eyes: random idle blinks and glances, eyes that
follow the person you're tracking, trail scanning while returning, and a
flashing SOS status. The RGB backlight colour changes with the mission state.

The driver finds either the older RGB address `0x62` or the newer V5 address
`0x30`. Enable I2C on the Pi, and set `LCD_I2C_DEVICE` if the bus device has
another name. The LCD needs 5 V. The V5 works with 3.3 V I2C, but older
versions may need a level shifter before you connect them to Pi GPIO. If the
LCD is wired to the Uno instead, the Uno draws the faces and this node just
keeps retrying.

### Audio

The Docker image generates local WAV files at build time with offline eSpeak
NG, including the SOS phrase. ALSA plays them through the Pi's sound device,
mounted at `/dev/snd`. S1 plays a start chime, S2 plays periodic beeps, and S3
loops the spoken alert. If the detector sees a person within 3 m, S3 switches
to a louder recording with a shorter pause. After reaching the trailhead in
S3, the rover slowly approaches the nearest detected person and stops about
1 m away.

### Still to do

Follow, return, SOS seeking, the display and audio all still need
calibration and testing on the assembled rover.

Use the same `ROS_DOMAIN_ID` on every machine that talks to the robot. Host
networking only works on Linux, and Tailscale on its own won't carry DDS
discovery between networks.

## Tests

```sh
make test
```

This runs the Python tests in `tests/` and the Rust tests in
`arduino_uno_rs/brain`. Neither needs ROS, Docker or any hardware. The bridge
tests replace ROS with small fakes and the Uno with a pseudo-terminal, then
check the exact bytes the bridge writes. One shared test vector makes sure
the Python encoder and the Rust decoder agree on the link format. You need
rustup for the Rust half (see [arduino_uno_rs/README.md](arduino_uno_rs/README.md#build-and-flash)).

## Reproducibility

The base image is pinned by digest in `.env`. `packages.lock.txt` records the
apt package versions from the last install, but rebuilds don't enforce them.
To copy an exact build to another robot with the same architecture:

```sh
sudo docker save local/ros2-robot:jazzy | gzip > ros2-jazzy-image.tar.gz
# on the other robot, inside this repo:
gunzip -c ros2-jazzy-image.tar.gz | sudo docker load
sudo docker compose up -d --no-build
```

## Validation status

This was written on macOS. Shell syntax is checked locally and `make test`
passes. The Uno firmware links with avr-gcc and uses 14.3 KB of 32 KB flash
and 753 B of 2 KB RAM. None of this has run on the robot yet: not the Docker
builds (including `flash_uno.sh`), not the systemd timer, and not the
hardware.
