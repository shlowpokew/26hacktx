//! The glorified state machine. The Pi is in charge: its follow, backtrack
//! and SOS controllers pick the wheel PWM. The Uno passes that through, but
//! refuses to drive into something the LiDAR says is too close, and stops on
//! its own if the Pi goes quiet.
//!
//! ```text
//!  Boot ──1.5 s──► NoLink ◄────────── any state, no frame for 500 ms
//!                    │ frame arrives
//!                    ▼
//!                  Idle ◄──────────── Pi commands 0, 0
//!                    │ Pi commands motion
//!                    ▼
//!                 Driving ──heading into something / scan stale──► Blocked
//!                    ▲                                                 │
//!                    └────────── path clear again (+10 cm margin) ────┘
//! ```
//!
//! "Heading into" depends on direction: forward checks the front sectors,
//! reverse checks the rear ones, and spinning in place is always allowed.
//!
//! The LiDAR filter drops returns under 15 cm, so an obstacle that gets that
//! close vanishes from the scan. Once a direction is blocked it stays latched:
//! "no return" there counts as too close until a real reading shows the extra
//! margin, or the rover spins or backs away (which changes what's in front).

use crate::protocol::{Command, Mission, NO_RETURN};

pub const BOOT_MS: u32 = 1500;
pub const LINK_TIMEOUT_MS: u32 = 500;

// Last line of defence; the Pi's controllers already stop at 25-30 cm.
// The LiDAR filter drops returns under 15 cm, so these must stay above that.
const STOP_AHEAD_CM: u16 = 20;
const STOP_REAR_CM: u16 = 20;
const STOP_CORNER_CM: u16 = 17;
const RELEASE_MARGIN_CM: u16 = 10;
pub const NOTICE_CM: u16 = 50;

/// |left + right| at or below this counts as spinning in place.
const SPIN_SUM: i16 = 60;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Boot,
    NoLink,
    Idle,
    Driving,
    Blocked,
}

impl State {
    pub fn name(self) -> &'static str {
        match self {
            State::Boot => "boot",
            State::NoLink => "nolink",
            State::Idle => "idle",
            State::Driving => "driving",
            State::Blocked => "blocked",
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Drive {
    pub left: i16,
    pub right: i16,
}

impl Drive {
    pub const STOP: Drive = Drive { left: 0, right: 0 };

    pub fn heading(self) -> Heading {
        let sum = self.left + self.right;
        if self == Drive::STOP {
            Heading::Still
        } else if sum > SPIN_SUM {
            Heading::Forward
        } else if sum < -SPIN_SUM {
            Heading::Reverse
        } else if self.right > self.left {
            Heading::SpinLeft
        } else {
            Heading::SpinRight
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Heading {
    Still,
    Forward,
    Reverse,
    SpinLeft,
    SpinRight,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Side {
    Front,
    Rear,
}

/// The four distances the stop rule looks at, each latched on its own so a
/// corner that never saw anything can't hold the rover up.
const FRONT_MIDDLE: usize = 0;
const FRONT_CORNER: usize = 1;
const REAR_MIDDLE: usize = 2;
const REAR_CORNER: usize = 3;

pub struct Brain {
    state: State,
    entered_at: u32,
    last_frame_at: Option<u32>,
    command: Command,
    latched: [bool; 4],
}

impl Brain {
    pub fn new(now: u32) -> Self {
        Brain {
            state: State::Boot,
            entered_at: now,
            last_frame_at: None,
            command: Command::NONE,
            latched: [false; 4],
        }
    }

    pub fn on_command(&mut self, now: u32, command: Command) {
        self.command = command;
        self.last_frame_at = Some(now);
    }

    pub fn state(&self) -> State {
        self.state
    }

    pub fn command(&self) -> &Command {
        &self.command
    }

    pub fn mission(&self) -> Mission {
        self.command.mission
    }

    /// What the Pi asked for, clamped to the PWM range.
    pub fn requested(&self) -> Drive {
        Drive {
            left: self.command.left.clamp(-255, 255),
            right: self.command.right.clamp(-255, 255),
        }
    }

    pub fn link_ok(&self, now: u32) -> bool {
        self.last_frame_at
            .is_some_and(|at| now.wrapping_sub(at) <= LINK_TIMEOUT_MS)
    }

    pub fn ahead_cm(&self) -> u16 {
        self.command.sectors_cm[0]
    }

    pub fn closest_cm(&self) -> u16 {
        self.command.sectors_cm.iter().copied().min().unwrap_or(u16::MAX)
    }

    /// Run the machine and return the wheel PWM to apply.
    pub fn tick(&mut self, now: u32) -> Drive {
        let next = self.next_state(now);
        if next != self.state {
            self.state = next;
            self.entered_at = now;
        }
        if self.link_ok(now) && self.command.scan_fresh {
            for i in 0..4 {
                self.latched[i] = self.too_close(i);
            }
        }
        if self.state != State::Driving {
            return Drive::STOP;
        }
        let keep = match self.requested().heading() {
            Heading::Forward => [true, true, false, false],
            Heading::Reverse => [false, false, true, true],
            _ => [false; 4],
        };
        for (latched, keep) in self.latched.iter_mut().zip(keep) {
            *latched &= keep;
        }
        self.requested()
    }

    fn next_state(&self, now: u32) -> State {
        if self.state == State::Boot && now.wrapping_sub(self.entered_at) < BOOT_MS {
            return State::Boot;
        }
        if !self.link_ok(now) {
            return State::NoLink;
        }
        let toward = match self.requested().heading() {
            Heading::Still => return State::Idle,
            Heading::Forward => Some(Side::Front),
            Heading::Reverse => Some(Side::Rear),
            Heading::SpinLeft | Heading::SpinRight => None,
        };
        let blocked = toward.is_some_and(|side| self.blocked_toward(side));
        if !self.command.scan_fresh || blocked {
            State::Blocked
        } else {
            State::Driving
        }
    }

    fn blocked_toward(&self, side: Side) -> bool {
        match side {
            Side::Front => self.too_close(FRONT_MIDDLE) || self.too_close(FRONT_CORNER),
            Side::Rear => self.too_close(REAR_MIDDLE) || self.too_close(REAR_CORNER),
        }
    }

    fn too_close(&self, which: usize) -> bool {
        let s = &self.command.sectors_cm;
        let (cm, limit) = match which {
            FRONT_MIDDLE => (s[0], STOP_AHEAD_CM),
            FRONT_CORNER => (s[1].min(s[7]), STOP_CORNER_CM),
            REAR_MIDDLE => (s[4], STOP_REAR_CM),
            _ => (s[3].min(s[5]), STOP_CORNER_CM),
        };
        if self.latched[which] {
            // Demand extra room so it doesn't stutter at the threshold, and
            // treat "nothing seen" as the obstacle having slipped under 15 cm.
            cm == NO_RETURN || cm < limit + RELEASE_MARGIN_CM
        } else {
            cm < limit
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FAR: u16 = NO_RETURN;
    const OPEN: [u16; 8] = [FAR; 8];

    fn cmd(left: i16, right: i16, sectors_cm: [u16; 8]) -> Command {
        Command { left, right, sectors_cm, scan_fresh: true, ..Command::NONE }
    }

    /// A brain past Boot that has just received `c`.
    fn booted(c: Command) -> (Brain, u32) {
        let mut brain = Brain::new(0);
        let t = BOOT_MS;
        brain.on_command(t, c);
        brain.tick(t);
        (brain, t)
    }

    fn step(brain: &mut Brain, t: u32, c: Command) -> Drive {
        brain.on_command(t, c);
        brain.tick(t)
    }

    #[test]
    fn boots_then_waits_for_the_pi() {
        let mut brain = Brain::new(0);
        assert_eq!(brain.tick(10), Drive::STOP);
        assert_eq!(brain.state(), State::Boot);
        brain.tick(BOOT_MS);
        assert_eq!(brain.state(), State::NoLink);
    }

    #[test]
    fn passes_the_pi_command_through() {
        let (mut brain, t) = booted(cmd(0, 0, OPEN));
        assert_eq!(brain.state(), State::Idle);
        assert_eq!(step(&mut brain, t + 50, cmd(150, 130, OPEN)), Drive { left: 150, right: 130 });
        assert_eq!(brain.state(), State::Driving);
    }

    #[test]
    fn clamps_out_of_range_pwm() {
        let (mut brain, t) = booted(cmd(0, 0, OPEN));
        assert_eq!(step(&mut brain, t, cmd(900, -900, OPEN)), Drive { left: 255, right: -255 });
    }

    #[test]
    fn stale_link_stops() {
        let (mut brain, t) = booted(cmd(150, 150, OPEN));
        assert_eq!(brain.state(), State::Driving);
        assert_eq!(brain.tick(t + LINK_TIMEOUT_MS + 1), Drive::STOP);
        assert_eq!(brain.state(), State::NoLink);
    }

    #[test]
    fn stale_scan_blocks_motion() {
        let (brain, _) = booted(Command { scan_fresh: false, ..cmd(150, 150, OPEN) });
        assert_eq!(brain.state(), State::Blocked);
    }

    #[test]
    fn blocks_forward_into_a_wall_but_allows_reverse_and_spin() {
        let wall_ahead = [19, 60, FAR, FAR, FAR, FAR, FAR, 60];
        let (mut brain, t) = booted(cmd(150, 150, wall_ahead));
        assert_eq!(brain.state(), State::Blocked);
        assert_eq!(brain.tick(t), Drive::STOP);

        let back = step(&mut brain, t + 50, cmd(-120, -120, wall_ahead));
        assert_eq!(back, Drive { left: -120, right: -120 });
        let spin = step(&mut brain, t + 100, cmd(-120, 120, wall_ahead));
        assert_eq!(spin, Drive { left: -120, right: 120 });
    }

    #[test]
    fn front_corner_counts() {
        let (brain, _) = booted(cmd(150, 150, [FAR, 16, FAR, FAR, FAR, FAR, FAR, FAR]));
        assert_eq!(brain.state(), State::Blocked);
    }

    #[test]
    fn blocks_reversing_into_something_behind() {
        let (brain, _) = booted(cmd(-120, -120, [FAR, FAR, FAR, FAR, 18, FAR, FAR, FAR]));
        assert_eq!(brain.state(), State::Blocked);
    }

    #[test]
    fn release_needs_extra_margin() {
        let at = |cm| [cm, FAR, FAR, FAR, FAR, FAR, FAR, FAR];
        let (mut brain, t) = booted(cmd(150, 150, at(19)));
        assert_eq!(brain.state(), State::Blocked);
        step(&mut brain, t + 50, cmd(150, 150, at(25)));
        assert_eq!(brain.state(), State::Blocked, "25 cm is past stop but inside the margin");
        step(&mut brain, t + 100, cmd(150, 150, at(31)));
        assert_eq!(brain.state(), State::Driving);
    }

    #[test]
    fn obstacle_vanishing_under_min_range_stays_blocked() {
        let at = |cm| [cm, FAR, FAR, FAR, FAR, FAR, FAR, FAR];
        let (mut brain, t) = booted(cmd(150, 150, at(19)));
        assert_eq!(brain.state(), State::Blocked);
        step(&mut brain, t + 50, cmd(150, 150, at(FAR)));
        assert_eq!(brain.state(), State::Blocked, "it went under 15 cm, not away");
    }

    #[test]
    fn spinning_away_clears_the_latch() {
        let at = |cm| [cm, FAR, FAR, FAR, FAR, FAR, FAR, FAR];
        let (mut brain, t) = booted(cmd(150, 150, at(19)));
        step(&mut brain, t + 50, cmd(-120, 120, at(FAR)));
        assert_eq!(brain.state(), State::Driving);
        step(&mut brain, t + 100, cmd(150, 150, at(FAR)));
        assert_eq!(brain.state(), State::Driving, "open field ahead after the turn");
    }

    #[test]
    fn open_field_never_latches() {
        let (mut brain, t) = booted(cmd(0, 0, OPEN));
        step(&mut brain, t + 50, Command { scan_fresh: false, ..cmd(150, 150, OPEN) });
        assert_eq!(brain.state(), State::Blocked, "stale scan");
        step(&mut brain, t + 100, cmd(150, 150, OPEN));
        assert_eq!(brain.state(), State::Driving);
    }

    #[test]
    fn heading_classification() {
        assert_eq!(Drive { left: 150, right: 150 }.heading(), Heading::Forward);
        assert_eq!(Drive { left: 200, right: 20 }.heading(), Heading::Forward);
        assert_eq!(Drive { left: -100, right: -100 }.heading(), Heading::Reverse);
        assert_eq!(Drive { left: -120, right: 120 }.heading(), Heading::SpinLeft);
        assert_eq!(Drive { left: 100, right: -80 }.heading(), Heading::SpinRight);
        assert_eq!(Drive::STOP.heading(), Heading::Still);
    }

    #[test]
    fn survives_millis_wraparound() {
        let start = u32::MAX - 100;
        let mut brain = Brain::new(start);
        let now = start.wrapping_add(BOOT_MS + 10);
        assert_eq!(step(&mut brain, now, cmd(150, 150, OPEN)), Drive { left: 150, right: 150 });
    }
}
