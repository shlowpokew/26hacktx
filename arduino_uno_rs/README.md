# Arduino Uno firmware (Rust)

This is a glorified state machine for an Arduino Uno (ATmega328P). The Pi
decides where the rover goes. Every 50 ms it sends the Uno one frame over USB
with the wheel PWM, the mission state, and the closest LiDAR return in each of
eight directions. The Uno drives the DRV8833, won't drive into anything closer
than about 20 cm, and puts a kaomoji on a 16x2 I2C LCD showing what it's up
to.

```
/cmd_vel ──► arduino_bridge.py ──USB serial──► Uno ──► DRV8833 ──► motors
/scan ───────┘ (PID + sectors)    ◄── status    └────► LCD  \(^ω^)/
/mission/state ┘
```

| Path | What it is |
| --- | --- |
| `brain/` | All the logic: link protocol, state machine, faces. Plain `no_std` Rust, with unit tests that run on a laptop. |
| `firmware/` | The AVR binary: pins, timers, interrupts, LCD and motor drivers. It calls into `brain`. |
| `../arduino_bridge.py` | ROS 2 node on the Pi: `/cmd_vel` PID loop, LiDAR sectors, the USB link. |
| `../uno_link.py` | The parts of the bridge that don't need ROS: frame encoding, LiDAR sectors, command parsing. |
| `../udev/99-breadcrumbs-uno.rules` | Names the Uno `/dev/uno` on the Pi. |

## Wiring

The main [README](../README.md#wiring-and-pin-layout) has full pin diagrams
for the Uno, the DRV8833 and the Pi. Here's what the Uno connects to:

| Uno | Goes to |
| --- | --- |
| USB | Pi (power, plus the serial link at 57600 baud) |
| A4 / A5 | LCD SDA / SCL |
| 5V / GND | LCD VCC / GND |
| D5 / D6 | DRV8833 IN1 / IN2 (left motor) |
| D9 / D10 | DRV8833 IN3 / IN4 (right motor) |
| D8 | DRV8833 EEP (sleep) |
| GND | DRV8833 GND and battery − |

The 6×AA pack's + goes to the DRV8833's VCC and nowhere else, never to the
Uno. Leave D0 and D1 unconnected; the USB serial link uses them.

### Which LCD

Any 16x2 character LCD on I2C should work. The firmware probes the bus at
boot and prints what it found (`lcd=backpack addr=39` means 0x27).

The generic "1602 I2C" LCD from kits (MLH hardware kits, Elegoo and so on) is
the blue or green one with a small board soldered to the back with four pins:
GND, VCC, SDA, SCL. It shows up at 0x27 or 0x3F. Its backlight is just on or
off, so the colours in the table below don't apply. If the screen lights up
but stays blank, or shows a row of solid boxes, turn the blue trimmer pot on
the little board. That's the contrast, and it often ships turned all the way
down.

The Seeed Grove 16x2 LCD, RGB or plain, works too, at 0x3E.

A 16-pin LCD without the backpack board needs six or more wires in parallel
mode, and that isn't supported. OLEDs aren't either.

## Build and flash

### On the robot, with Docker

You don't need a toolchain for this.

```sh
bash flash_uno.sh                         # from the repo root: build and flash
bash flash_uno.sh --no-default-features   # dry run: reports PWM, never drives the motors
```

It builds the `Dockerfile` in this folder (Ubuntu, the pinned Rust nightly,
and avr-gcc), converts the ELF to a hex file, and flashes it with avrdude. The
ROS container holds the Uno's serial port, so it gets stopped while flashing
and started again afterwards. Flashing resets the Uno, so keep the wheels off
the ground. The main [README](../README.md#flashing) has more.

### On a laptop, with a local toolchain

Set up once:

```sh
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
cargo install ravedude
# macOS
brew tap osx-cross/avr && brew install avr-gcc avrdude
# Raspberry Pi OS / Ubuntu
sudo apt install gcc-avr avr-libc avrdude
```

`firmware/rust-toolchain.toml` pins the nightly that avr-hal needs, and rustup
fetches it on the first build.

```sh
cd arduino_uno_rs/firmware
cargo run --release                         # build, flash, open the serial console
cargo run --release --no-default-features   # dry run
```

ravedude finds the Uno by itself. If it grabs the wrong port, set
`RAVEDUDE_PORT`.

### Tests

The logic tests run on a laptop without a board:

```sh
cd arduino_uno_rs/brain && cargo test     # or `make test` at the repo root
```

## State machine

```
 Boot ──1.5 s──► NoLink ◄────────── any state, no frame for 500 ms
                   │ frame arrives
                   ▼
                 Idle ◄──────────── Pi commands 0, 0
                   │ Pi commands motion
                   ▼
                Driving ──heading into something / scan stale──► Blocked
                   ▲                                                 │
                   └────────── path clear again (+10 cm margin) ────┘
```

In Driving, the Uno applies the Pi's PWM as is. Speed is the Pi's PID loop's
job.

Whether it's Blocked depends on which way the Pi wants to go. Going forward,
it blocks if something is closer than 20 cm ahead or 17 cm at a front corner.
Reversing uses the same limits behind. Spinning in place is always allowed,
since that's how the rover turns away from a wall. It also blocks when the
Pi says its LiDAR data is stale.

Normally the Uno never has to step in, because the Pi's own controllers stop
at 25 to 30 cm.

There's one catch with the LiDAR. The filter on the Pi drops returns under
15 cm, so anything that gets closer than that vanishes from the scan. To deal
with that, a blocked direction stays latched: "no return" there still counts
as blocked until a real reading shows 10 cm of extra room, or the Pi spins or
backs the rover away.

The thresholds are constants at the top of `brain/src/fsm.rs`.

The Uno also stops by itself if frames stop for 500 ms (the Pi crashed, or
the cable got pulled), or if the watchdog resets a hung loop after 1 s. A
reset leaves the motor pins off.

## Faces

| When | Face | Caption | Backlight (Grove RGB only) |
| --- | --- | --- | --- |
| Boot | `(o_o)` / `(^_^)/` | hello!! ✧ | white |
| No frames from the Pi | `(・_・ )?` | where's the pi? | purple |
| Idle | `(=^・ω・^=)`, blinks | nap time | blue |
| Idle, something within 50 cm | `(°o°)!` | oh hi! ♥ | pink |
| Follow, standing by | `(・ω・)♥` | with you ♥ | green |
| Backtrack, standing by | `(-_-)` | thinking | yellow |
| Driving forward | `\(^ω^)/` / `~(^ω^)~` | follow ♥ / going home / manual | by mission |
| Spinning | `<(・ω・ )` or `( ・ω・)>` | same | by mission |
| Reversing | `(°□°;)` | same | by mission |
| Blocked | `(;_;)` / `(T_T)` | blocked! | red |
| Pi safety warning | `(・_・;)` | pi says stop | orange |
| SOS | `(;°□°)!` / `!! SOS !!` | please help me! | flashing red/white |

The right end of the bottom row shows the distance straight ahead.

The LCD only knows ASCII plus a page of Japanese characters, and on that page
`\` shows up as `¥` and `~` as `→`. So the faces in `brain/src/faces.rs` are
written as normal Unicode strings, and `ω ♥ \ ~ ° ・ ✧ □` get drawn as custom
glyphs in the LCD's 8 CGRAM slots. Any other non-ASCII character shows as `?`.
To swap a glyph, edit `CUSTOM` and `GLYPHS` in that file.

## Link protocol

`A5 5A | 02 | 16 | payload (22 bytes) | crc8`

| Payload bytes | Field |
| --- | --- |
| 1 | flags: bit 0 scan fresh, bit 1 Pi safety warning |
| 1 | mission: 0 idle, 1 follow, 2 backtrack, 3 SOS |
| 2 + 2 | left, right PWM, `i16` little-endian, -255..255 |
| 8 × 2 | closest return per 45° sector, `u16` LE cm, `0xFFFF` = none |

Sector 0 is straight ahead, and they go counter-clockwise: 2 is left, 4 is
behind, 6 is right. The CRC is CRC-8/SMBUS over everything after the sync
bytes. The Rust and Python tests both check the same sample frame, so the
decoder here and the encoder in `uno_link.py` can't drift apart without a test
failing.

The Uno sends back one line of text every 200 ms, like
`st=driving mission=follow l=150 r=130 ahead=212 near=88 bad=0`. `bad` counts
corrupted frames.

## Validation status

`make test` at the repo root passes on macOS. That's 36 Rust tests (protocol,
state machine, faces, ring buffer, LCD backpack encoding) and 39 Python tests
(`uno_link.py` and `arduino_bridge.py`).

The firmware builds and links with avr-gcc 7.3 for the ATmega328P. With
motors it uses 14.3 KB of 32 KB flash and 753 B of 2 KB RAM; without them,
13.8 KB of flash.

The Docker flashing image (`Dockerfile` and `../flash_uno.sh`) hasn't been
built yet, and nothing has been flashed or run on a real Uno, LCD, motors or
the Pi stack. Do the first power-up as a bench test with the wheels off the
ground.
