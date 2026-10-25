//! Pi -> Uno link over USB serial.
//!
//! Frame: `A5 5A | type | len | payload[len] | crc8(type, len, payload)`.
//!
//! The only message is `MSG_CONTROL`, sent by arduino_bridge.py every 50 ms:
//!
//! | bytes | field |
//! | --- | --- |
//! | 1 | flags: bit 0 = scan is fresh, bit 1 = Pi has a safety warning |
//! | 1 | mission: 0 idle, 1 follow, 2 backtrack, 3 SOS |
//! | 2 + 2 | left, right wheel PWM, `i16` LE, -255..255 |
//! | 8 x 2 | closest LiDAR return per 45 degree sector, `u16` LE cm |
//!
//! Sector 0 is straight ahead and they go counter-clockwise (ROS convention),
//! so 2 is left, 4 is behind, 6 is right. `NO_RETURN` means the sector had no
//! valid points.

pub const SYNC0: u8 = 0xA5;
pub const SYNC1: u8 = 0x5A;
pub const MSG_CONTROL: u8 = 0x02;
pub const SECTORS: usize = 8;
pub const CONTROL_LEN: usize = 2 + 4 + 2 * SECTORS;
pub const FRAME_LEN: usize = 4 + CONTROL_LEN + 1;
pub const NO_RETURN: u16 = 0xFFFF;

const FLAG_SCAN_FRESH: u8 = 0x01;
const FLAG_WARNING: u8 = 0x02;
const MAX_PAYLOAD: usize = CONTROL_LEN;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mission {
    Idle,
    Follow,
    Backtrack,
    Sos,
}

impl Mission {
    fn from_byte(byte: u8) -> Mission {
        match byte {
            1 => Mission::Follow,
            2 => Mission::Backtrack,
            3 => Mission::Sos,
            _ => Mission::Idle,
        }
    }

    fn to_byte(self) -> u8 {
        match self {
            Mission::Idle => 0,
            Mission::Follow => 1,
            Mission::Backtrack => 2,
            Mission::Sos => 3,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Mission::Idle => "idle",
            Mission::Follow => "follow",
            Mission::Backtrack => "backtrack",
            Mission::Sos => "sos",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Command {
    pub scan_fresh: bool,
    pub warning: bool,
    pub mission: Mission,
    pub left: i16,
    pub right: i16,
    pub sectors_cm: [u16; SECTORS],
}

impl Command {
    /// What the Uno assumes before the Pi has said anything: stop, see nothing.
    pub const NONE: Command = Command {
        scan_fresh: false,
        warning: false,
        mission: Mission::Idle,
        left: 0,
        right: 0,
        sectors_cm: [NO_RETURN; SECTORS],
    };
}

/// CRC-8/SMBUS: polynomial 0x07, initial value 0.
pub fn crc8(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0, |crc, &byte| crc8_step(crc, byte))
}

fn crc8_step(crc: u8, byte: u8) -> u8 {
    let mut crc = crc ^ byte;
    for _ in 0..8 {
        crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x07 } else { crc << 1 };
    }
    crc
}

pub fn encode(command: &Command) -> [u8; FRAME_LEN] {
    let mut frame = [0u8; FRAME_LEN];
    frame[0] = SYNC0;
    frame[1] = SYNC1;
    frame[2] = MSG_CONTROL;
    frame[3] = CONTROL_LEN as u8;
    frame[4] = if command.scan_fresh { FLAG_SCAN_FRESH } else { 0 }
        | if command.warning { FLAG_WARNING } else { 0 };
    frame[5] = command.mission.to_byte();
    frame[6..8].copy_from_slice(&command.left.to_le_bytes());
    frame[8..10].copy_from_slice(&command.right.to_le_bytes());
    for (i, cm) in command.sectors_cm.iter().enumerate() {
        frame[10 + 2 * i..12 + 2 * i].copy_from_slice(&cm.to_le_bytes());
    }
    frame[FRAME_LEN - 1] = crc8(&frame[2..FRAME_LEN - 1]);
    frame
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Step {
    Sync0,
    Sync1,
    Kind,
    Len,
    Payload,
    Crc,
}

/// Byte-at-a-time frame parser. Feed it every received byte; it hands back a
/// `Command` when a complete frame with a good CRC arrives and resyncs on junk.
pub struct Parser {
    step: Step,
    kind: u8,
    len: u8,
    filled: usize,
    payload: [u8; MAX_PAYLOAD],
    pub bad_frames: u16,
}

impl Parser {
    pub const fn new() -> Self {
        Parser {
            step: Step::Sync0,
            kind: 0,
            len: 0,
            filled: 0,
            payload: [0; MAX_PAYLOAD],
            bad_frames: 0,
        }
    }

    pub fn push(&mut self, byte: u8) -> Option<Command> {
        match self.step {
            Step::Sync0 => {
                if byte == SYNC0 {
                    self.step = Step::Sync1;
                }
            }
            Step::Sync1 => {
                self.step = match byte {
                    SYNC1 => Step::Kind,
                    SYNC0 => Step::Sync1,
                    _ => Step::Sync0,
                };
            }
            Step::Kind => {
                self.kind = byte;
                self.step = Step::Len;
            }
            Step::Len => {
                if byte as usize > MAX_PAYLOAD {
                    self.bad_frames = self.bad_frames.wrapping_add(1);
                    self.step = Step::Sync0;
                } else {
                    self.len = byte;
                    self.filled = 0;
                    self.step = if byte == 0 { Step::Crc } else { Step::Payload };
                }
            }
            Step::Payload => {
                self.payload[self.filled] = byte;
                self.filled += 1;
                if self.filled == self.len as usize {
                    self.step = Step::Crc;
                }
            }
            Step::Crc => {
                self.step = Step::Sync0;
                let body = &self.payload[..self.len as usize];
                let crc = body.iter().fold(crc8(&[self.kind, self.len]), |c, &b| crc8_step(c, b));
                if crc != byte {
                    self.bad_frames = self.bad_frames.wrapping_add(1);
                    return None;
                }
                if self.kind == MSG_CONTROL && body.len() == CONTROL_LEN {
                    return Some(decode(body));
                }
            }
        }
        None
    }
}

impl Default for Parser {
    fn default() -> Self {
        Self::new()
    }
}

fn decode(payload: &[u8]) -> Command {
    let mut sectors_cm = [NO_RETURN; SECTORS];
    for (i, cm) in sectors_cm.iter_mut().enumerate() {
        *cm = u16::from_le_bytes([payload[6 + 2 * i], payload[7 + 2 * i]]);
    }
    Command {
        scan_fresh: payload[0] & FLAG_SCAN_FRESH != 0,
        warning: payload[0] & FLAG_WARNING != 0,
        mission: Mission::from_byte(payload[1]),
        left: i16::from_le_bytes([payload[2], payload[3]]),
        right: i16::from_le_bytes([payload[4], payload[5]]),
        sectors_cm,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feed(parser: &mut Parser, bytes: &[u8]) -> Vec<Command> {
        bytes.iter().filter_map(|&b| parser.push(b)).collect()
    }

    const SAMPLE: Command = Command {
        scan_fresh: true,
        warning: false,
        mission: Mission::Follow,
        left: 140,
        right: -120,
        sectors_cm: [120, 300, NO_RETURN, 45, 80, 999, 20, 65],
    };

    #[test]
    fn round_trips() {
        let mut parser = Parser::new();
        assert_eq!(feed(&mut parser, &encode(&SAMPLE)), vec![SAMPLE]);
    }

    #[test]
    fn matches_python_encoder() {
        // arduino_bridge.encode_control(True, False, 1, 140, -120,
        //                               [120, 300, 0xFFFF, 45, 80, 999, 20, 65])
        let python = [
            0xa5, 0x5a, 0x02, 0x16, 0x01, 0x01, 0x8c, 0x00, 0x88, 0xff, 0x78, 0x00, 0x2c, 0x01,
            0xff, 0xff, 0x2d, 0x00, 0x50, 0x00, 0xe7, 0x03, 0x14, 0x00, 0x41, 0x00, 0xdf,
        ];
        assert_eq!(encode(&SAMPLE), python);
    }

    #[test]
    fn rejects_bad_crc_then_recovers() {
        let mut parser = Parser::new();
        let mut broken = encode(&SAMPLE);
        broken[7] ^= 0x10;
        assert!(feed(&mut parser, &broken).is_empty());
        assert_eq!(parser.bad_frames, 1);
        assert_eq!(feed(&mut parser, &encode(&SAMPLE)), vec![SAMPLE]);
    }

    #[test]
    fn resyncs_through_garbage_and_repeated_sync() {
        let mut parser = Parser::new();
        let mut stream = vec![0x00, 0xA5, 0x13, 0xA5, 0xA5];
        stream.extend_from_slice(&encode(&SAMPLE)[1..]);
        assert_eq!(feed(&mut parser, &stream), vec![SAMPLE]);
    }

    #[test]
    fn oversized_length_is_dropped() {
        let mut parser = Parser::new();
        assert!(feed(&mut parser, &[SYNC0, SYNC1, MSG_CONTROL, 200]).is_empty());
        assert_eq!(parser.bad_frames, 1);
        assert_eq!(feed(&mut parser, &encode(&SAMPLE)), vec![SAMPLE]);
    }

    #[test]
    fn unknown_mission_byte_reads_as_idle() {
        let mut frame = encode(&SAMPLE);
        frame[5] = 9;
        frame[FRAME_LEN - 1] = crc8(&frame[2..FRAME_LEN - 1]);
        let got = feed(&mut Parser::new(), &frame);
        assert_eq!(got[0].mission, Mission::Idle);
    }
}
