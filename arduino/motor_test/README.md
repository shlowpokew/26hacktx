# One-motor bench test

Target: Arduino UNO R4 WiFi and DRV8833 module.

| Arduino | Driver |
| --- | --- |
| D5 | IN1 |
| D6 | IN2 |
| D8 | EEP / nSLEEP (not ULT / nFAULT) |
| GND | GND |

Connect the motor leads to OUT1 and OUT2 for the IN1/IN2 channel. Connect a
suitable motor supply to the driver's motor supply input and GND; do not
power the motor from an Arduino I/O pin. Match the supply to both the motor
and module ratings. Keep the wheel clear of the bench for this test.

## VS Code + PlatformIO over SSH

Connect the Arduino by USB-A to USB-C data cable to the Pi. In desktop VS Code,
install Microsoft's Remote - SSH extension, connect to your Pi account, and
open ~/etc/ros2/arduino/motor_test on the Pi (adjust for your checkout location).
Install PlatformIO IDE in the SSH: Pi extension host. Run PlatformIO: New
Terminal from the command palette. Build and upload on the Pi host, outside
the ROS Docker container:

```sh
pio run
pio device list
pio run --target upload --upload-port /dev/ttyACM0
```

Replace /dev/ttyACM0 with the Arduino's actual port from `pio device list`.
The first build downloads the board platform, Arduino framework and tools.
No extra sketch libraries are required. PlatformIO uses the existing .ino
file directly through platformio.ini; do not create a second copy in src/.

If access to the serial port is denied on Raspberry Pi OS/Ubuntu, run:

```sh
sudo usermod -aG dialout "$USER"
```

Then end the VS Code SSH connection and reconnect so the new group takes
effect. Do not run PlatformIO with sudo. Close serial monitors and any ROS
serial bridge before uploading. If the board is not detected, double-press
RESET to enter bootloader mode, list ports again, and retry the upload with
the bootloader's port.

Optional serial monitor (Ctrl+C exits):

```sh
pio device monitor --port /dev/ttyACM0 --baud 115200
```

Alternatively, Arduino IDE can open motor_test.ino directly with the
Arduino UNO R4 WiFi board selected. ROS and Docker are not needed for this test.

After startup/reset, the sketch waits three seconds, applies about 25% PWM
for two seconds, then removes motor drive. It runs once per reset/power-up;
the shaft can coast after drive stops. Pin 8 is a digital enable, not a PWM pin.
Optional Serial Monitor output uses 115200 baud; opening the monitor may reset
the board, so keep the wheel clear whenever connecting or resetting it.

If the motor only hums, disconnect motor power and try MOTOR_PWM = 90 rather
than leaving it stalled. PWM duty is not a guaranteed speed. This test does
not read encoders or verify Pi-to-Arduino serial communication.

Control behavior reference: https://www.ti.com/lit/ds/symlink/drv8833.pdf

The sketch has not been compiled or run on hardware in this workspace.
