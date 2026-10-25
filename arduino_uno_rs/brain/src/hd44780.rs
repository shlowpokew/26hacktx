//! Byte encoding for an HD44780 LCD behind a PCF8574 I2C backpack, the
//! common "1602 I2C" kit module. Each byte written to the PCF8574 sets its
//! eight pins, wired on the usual backpack as:
//!
//! P0 RS, P1 RW, P2 E, P3 backlight, P4..P7 LCD D4..D7.
//!
//! The LCD runs in 4-bit mode, so every LCD byte is two nibbles, each latched
//! by taking E high then low.

pub const RS: u8 = 0x01;
pub const EN: u8 = 0x04;
pub const BACKLIGHT: u8 = 0x08;

pub const CMD_FUNCTION_2LINE: u8 = 0x28;
pub const CMD_DISPLAY_ON: u8 = 0x0C;
pub const CMD_CLEAR: u8 = 0x01;
pub const CMD_ENTRY_LEFT: u8 = 0x06;

/// Pin states to clock in the high nibble of `nibble` (low bits ignored).
pub fn pulse(nibble: u8) -> [u8; 2] {
    let pins = (nibble & 0xF0) | BACKLIGHT;
    [pins | EN, pins]
}

/// A command byte (`data = false`) or character byte (`data = true`).
pub fn byte(value: u8, data: bool) -> [u8; 4] {
    let mode = if data { RS } else { 0 };
    let [hi_e, hi] = pulse(value);
    let [lo_e, lo] = pulse(value << 4);
    [hi_e | mode, hi | mode, lo_e | mode, lo | mode]
}

/// Set DDRAM address to the start of `row` (0 or 1).
pub fn row_command(row: u8) -> u8 {
    0x80 | if row == 0 { 0x00 } else { 0x40 }
}

/// Set CGRAM address for custom glyph `slot` (0..7).
pub fn glyph_command(slot: u8) -> u8 {
    0x40 | ((slot & 0x07) << 3)
}

/// Encode up to 16 character bytes into `out` for one I2C write; returns the
/// number of bytes used.
pub fn characters(text: &[u8], out: &mut [u8; 64]) -> usize {
    let count = text.len().min(16);
    for (chunk, &c) in out.chunks_exact_mut(4).zip(&text[..count]) {
        chunk.copy_from_slice(&byte(c, true));
    }
    count * 4
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn character_matches_liquidcrystal_i2c() {
        // 'A' = 0x41: high nibble 4, low nibble 1, RS and backlight set.
        assert_eq!(byte(b'A', true), [0x4D, 0x49, 0x1D, 0x19]);
    }

    #[test]
    fn command_has_rs_low() {
        assert_eq!(byte(CMD_CLEAR, false), [0x0C, 0x08, 0x1C, 0x18]);
        for pins in byte(CMD_FUNCTION_2LINE, false) {
            assert_eq!(pins & RS, 0);
        }
    }

    #[test]
    fn every_pulse_rises_then_falls_with_backlight_on() {
        for value in 0..=255u8 {
            let pins = byte(value, value % 2 == 0);
            assert_eq!(pins.map(|p| p & EN != 0), [true, false, true, false]);
            assert!(pins.iter().all(|p| p & BACKLIGHT != 0));
            assert_eq!(pins.map(|p| p & 0xF0), [value & 0xF0, value & 0xF0, value << 4, value << 4]);
        }
    }

    #[test]
    fn init_nibble_ignores_low_bits() {
        assert_eq!(pulse(0x3F), pulse(0x30));
        assert_eq!(pulse(0x20), [0x2C, 0x28]);
    }

    #[test]
    fn addresses() {
        assert_eq!(row_command(0), 0x80);
        assert_eq!(row_command(1), 0xC0);
        assert_eq!(glyph_command(0), 0x40);
        assert_eq!(glyph_command(7), 0x78);
    }

    #[test]
    fn characters_fill_four_bytes_each_and_cap_at_sixteen() {
        let mut out = [0u8; 64];
        assert_eq!(characters(b"hi", &mut out), 8);
        assert_eq!(out[..4], byte(b'h', true));
        assert_eq!(out[4..8], byte(b'i', true));
        assert_eq!(characters(&[b'x'; 20], &mut out), 64);
    }
}
