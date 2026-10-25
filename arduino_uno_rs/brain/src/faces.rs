//! Moe kaomoji for a 16x2 character LCD.
//!
//! The LCD only has ASCII plus a Japanese ROM page, and the ROM swaps `\` for
//! `¥` and `~` for `→`. So the faces are written as normal Unicode strings and
//! the few characters the ROM can't show are drawn as custom 5x8 glyphs in the
//! LCD's eight CGRAM slots.

use crate::fsm::{Brain, Heading, State, NOTICE_CM};
use crate::protocol::{Mission, NO_RETURN};

pub const COLS: usize = 16;

/// Characters that get a CGRAM slot. The slot number is the index.
const CUSTOM: [char; 8] = ['ω', '♥', '\\', '~', '°', '・', '✧', '□'];

/// 5x8 bitmaps for `CUSTOM`, top row first. Load into CGRAM at startup.
pub const GLYPHS: [[u8; 8]; 8] = [
    [0b00000, 0b00000, 0b00000, 0b10001, 0b10101, 0b10101, 0b01010, 0b00000], // ω
    [0b00000, 0b01010, 0b11111, 0b11111, 0b01110, 0b00100, 0b00000, 0b00000], // ♥
    [0b00000, 0b10000, 0b01000, 0b00100, 0b00010, 0b00001, 0b00000, 0b00000], // \
    [0b00000, 0b00000, 0b01000, 0b10101, 0b00010, 0b00000, 0b00000, 0b00000], // ~
    [0b01100, 0b10010, 0b10010, 0b01100, 0b00000, 0b00000, 0b00000, 0b00000], // °
    [0b00000, 0b00000, 0b00000, 0b01100, 0b01100, 0b00000, 0b00000, 0b00000], // ・
    [0b00100, 0b00100, 0b01010, 0b10001, 0b01010, 0b00100, 0b00100, 0b00000], // ✧
    [0b00000, 0b11111, 0b10001, 0b10001, 0b10001, 0b11111, 0b00000, 0b00000], // □
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Screen {
    pub top: [u8; COLS],
    pub bottom: [u8; COLS],
    /// Backlight colour; only the Grove RGB LCD uses it.
    pub rgb: [u8; 3],
}

struct Mood {
    face: &'static str,
    caption: &'static str,
    rgb: [u8; 3],
    show_distance: bool,
}

const WHITE: [u8; 3] = [120, 120, 120];
const RED: [u8; 3] = [200, 0, 0];
const GREEN: [u8; 3] = [0, 170, 40];
const YELLOW: [u8; 3] = [220, 150, 0];

fn mood(brain: &Brain, now: u32) -> Mood {
    let beat = (now / 500) % 2 == 0;
    let blink = now % 3700 < 180;
    let pick = |a, b| if beat { a } else { b };
    let m = |face, caption, rgb, show_distance| Mood { face, caption, rgb, show_distance };
    let mission = brain.mission();

    match brain.state() {
        State::Boot => m(pick("(o_o)", "(^_^)/"), "hello!! ✧", WHITE, false),
        State::NoLink => m(pick("(・_・ )?", "( ・_・)?"), "where's the pi?", [90, 0, 140], false),
        State::Blocked => m(pick("(;_;)", "(T_T)"), "blocked!", RED, true),
        _ if mission == Mission::Sos => {
            m(pick("(;°□°)!", "!! SOS !!"), "please help me!", if beat { RED } else { WHITE }, false)
        }
        State::Idle if brain.command().warning => m("(・_・;)", "pi says stop", [255, 70, 0], false),
        State::Idle => match mission {
            Mission::Follow => m(if blink { "(-ω-)♥" } else { "(・ω・)♥" }, "with you ♥", GREEN, true),
            Mission::Backtrack => m(pick("(-_-)", "(-_-).."), "thinking", YELLOW, true),
            _ if brain.closest_cm() < NOTICE_CM => m("(°o°)!", "oh hi! ♥", [255, 40, 120], true),
            _ => m(if blink { "(=^-ω-^=)" } else { "(=^・ω・^=)" }, "nap time", [0, 50, 150], true),
        },
        State::Driving => {
            let face = match brain.requested().heading() {
                Heading::Reverse => pick("(°□°;)", "(°□°;)!"),
                Heading::SpinLeft => pick("<(・ω・ )", "<<(・ω・)"),
                Heading::SpinRight => pick("( ・ω・)>", "(・ω・)>>"),
                Heading::Forward | Heading::Still => pick("\\(^ω^)/", "~(^ω^)~"),
            };
            match mission {
                Mission::Follow => m(face, "follow ♥", GREEN, true),
                Mission::Backtrack => m(face, "going home", YELLOW, true),
                _ => m(face, "manual", [0, 120, 200], true),
            }
        }
    }
}

pub fn screen(brain: &Brain, now: u32) -> Screen {
    let mood = mood(brain, now);
    let mut top = [b' '; COLS];
    let width = mood.face.chars().count().min(COLS);
    put(&mut top, (COLS - width) / 2, mood.face);

    let mut bottom = [b' '; COLS];
    put(&mut bottom, 0, mood.caption);
    if mood.show_distance {
        put_distance(&mut bottom[COLS - 5..], brain.ahead_cm());
    }
    Screen { top, bottom, rgb: mood.rgb }
}

/// LCD byte for a character: CGRAM slot, plain ASCII, or `?`.
pub fn lcd_code(c: char) -> u8 {
    if let Some(slot) = CUSTOM.iter().position(|&g| g == c) {
        slot as u8
    } else if c.is_ascii() && !c.is_ascii_control() {
        c as u8
    } else {
        b'?'
    }
}

fn put(row: &mut [u8], at: usize, text: &str) {
    for (cell, c) in row.iter_mut().skip(at).zip(text.chars()) {
        *cell = lcd_code(c);
    }
}

/// Five cells, right-aligned: `" 87cm"`, `"123cm"`, `"  far"`, `"   --"`.
fn put_distance(cells: &mut [u8], cm: u16) {
    cells.fill(b' ');
    if cm == NO_RETURN {
        cells[3..].copy_from_slice(b"--");
    } else if cm > 999 {
        cells[2..].copy_from_slice(b"far");
    } else {
        cells[3..].copy_from_slice(b"cm");
        let mut n = cm;
        let mut i = 2;
        loop {
            cells[i] = b'0' + (n % 10) as u8;
            n /= 10;
            if n == 0 || i == 0 {
                break;
            }
            i -= 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fsm::BOOT_MS;
    use crate::protocol::Command;

    fn text(row: &[u8; COLS]) -> String {
        row.iter()
            .map(|&b| if (b as usize) < CUSTOM.len() { CUSTOM[b as usize] } else { b as char })
            .collect()
    }

    fn brain_with(command: Command) -> Brain {
        let mut brain = Brain::new(0);
        brain.on_command(BOOT_MS, command);
        brain.tick(BOOT_MS);
        brain
    }

    fn command(mission: Mission, left: i16, right: i16, ahead: u16) -> Command {
        let mut sectors_cm = [NO_RETURN; 8];
        sectors_cm[0] = ahead;
        Command { scan_fresh: true, mission, left, right, sectors_cm, ..Command::NONE }
    }

    #[test]
    fn idle_cat_uses_custom_glyphs_and_centers() {
        let s = screen(&brain_with(command(Mission::Idle, 0, 0, NO_RETURN)), 1000);
        assert_eq!(text(&s.top), "   (=^・ω・^=)    ");
        assert_eq!(s.top[6], 5, "middle dot lives in CGRAM slot 5");
        assert_eq!(text(&s.bottom), "nap time      --");
    }

    #[test]
    fn following_forward_avoids_the_yen_sign() {
        let s = screen(&brain_with(command(Mission::Follow, 150, 150, 87)), 0);
        assert_eq!(text(&s.top), "    \\(^ω^)/     ");
        assert!(!s.top.contains(&b'\\'));
        assert_eq!(text(&s.bottom), "follow ♥    87cm");
    }

    #[test]
    fn spin_direction_shows_on_the_face() {
        let s = screen(&brain_with(command(Mission::Backtrack, -120, 120, 300)), 0);
        assert_eq!(text(&s.top), "    <(・ω・ )     ");
        assert_eq!(text(&s.bottom), "going home 300cm");
    }

    #[test]
    fn blocked_beats_mission() {
        let s = screen(&brain_with(command(Mission::Follow, 150, 150, 10 + 5)), 0);
        assert_eq!(text(&s.bottom), "blocked!    15cm");
    }

    #[test]
    fn idle_notices_someone_close() {
        let s = screen(&brain_with(command(Mission::Idle, 0, 0, 40)), 0);
        assert_eq!(text(&s.top), "     (°o°)!     ");
    }

    #[test]
    fn distances_format_in_five_cells() {
        let mut cells = [0u8; 5];
        for (cm, want) in [(5, "  5cm"), (123, "123cm"), (1000, "  far"), (NO_RETURN, "   --")] {
            put_distance(&mut cells, cm);
            assert_eq!(core::str::from_utf8(&cells).unwrap(), want);
        }
    }

    #[test]
    fn every_face_fits_and_has_known_chars() {
        let missions = [Mission::Idle, Mission::Follow, Mission::Backtrack, Mission::Sos];
        let drives = [(0, 0), (150, 150), (-120, -120), (-120, 120), (120, -120)];
        for mission in missions {
            for (left, right) in drives {
                for ahead in [10, 40, 500] {
                    for warning in [false, true] {
                        let c = Command { warning, ..command(mission, left, right, ahead) };
                        let brain = brain_with(c);
                        for now in [0, 600, 3700] {
                            let mood = mood(&brain, now);
                            let limit = if mood.show_distance { COLS - 6 } else { COLS };
                            assert!(mood.face.chars().count() <= COLS);
                            assert!(mood.caption.chars().count() <= limit, "{}", mood.caption);
                            for c in mood.face.chars().chain(mood.caption.chars()) {
                                assert!(c == '?' || lcd_code(c) != b'?', "{c:?} has no glyph");
                            }
                        }
                    }
                }
            }
        }
    }
}
