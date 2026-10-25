//! 16x2 character LCDs on I2C. Both kinds are an HD44780-style controller
//! underneath, so the faces and custom glyphs work the same on either:
//!
//! - Generic "1602 + I2C backpack" (the blue/green kit LCD with a little
//!   board on the back): a PCF8574 I/O expander at 0x27 or 0x3F driving the
//!   LCD in 4-bit mode. Backlight is on/off only.
//! - Seeed Grove 16x2 (RGB) LCD: AiP31068 at 0x3E, plus an RGB backlight chip
//!   at 0x62 (V4, PCA9633) or 0x30 (V5) on the RGB model. Same sequence as
//!   display_feedback.py on the Pi.

use embedded_hal::i2c::I2c;
use uno_brain::hd44780::{self, BACKLIGHT};

const GROVE_TEXT: u8 = 0x3E;
const GROVE_RGB_V5: u8 = 0x30;
const GROVE_RGB_V4: u8 = 0x62;
// Backpacks ship at 0x27 (PCF8574) or 0x3F (PCF8574A); the rest are jumper
// settings. 0x3E is checked as a Grove before the leftover backpack range.
const BACKPACK_COMMON: [u8; 2] = [0x27, 0x3F];
const BACKPACK_OTHER: [u8; 14] = [
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E,
];

type Bus = arduino_hal::I2c;
type Result<T = ()> = core::result::Result<T, arduino_hal::i2c::Error>;

#[derive(Clone, Copy)]
enum Kind {
    Grove { rgb: Option<u8> },
    Backpack { address: u8 },
}

pub struct Lcd {
    kind: Kind,
}

impl Lcd {
    /// Find whichever LCD is on the bus, initialise it, and load the glyphs.
    pub fn init(i2c: &mut Bus, glyphs: &[[u8; 8]; 8]) -> Result<Self> {
        let kind = detect(i2c).ok_or(arduino_hal::i2c::Error::AddressNack)?;
        let lcd = Lcd { kind };
        arduino_hal::delay_ms(50);
        match kind {
            Kind::Grove { rgb } => lcd.init_grove(i2c, rgb)?,
            Kind::Backpack { address } => lcd.init_backpack(i2c, address)?,
        }
        lcd.command(i2c, hd44780::CMD_FUNCTION_2LINE)?;
        lcd.command(i2c, hd44780::CMD_DISPLAY_ON)?;
        lcd.command(i2c, hd44780::CMD_CLEAR)?;
        arduino_hal::delay_ms(2);
        lcd.command(i2c, hd44780::CMD_ENTRY_LEFT)?;
        for (slot, glyph) in glyphs.iter().enumerate() {
            lcd.command(i2c, hd44780::glyph_command(slot as u8))?;
            lcd.data(i2c, glyph)?;
        }
        Ok(lcd)
    }

    /// For the boot log: which LCD was found and where.
    pub fn describe(&self) -> (&'static str, u8) {
        match self.kind {
            Kind::Grove { rgb: Some(_) } => ("grove_rgb", GROVE_TEXT),
            Kind::Grove { rgb: None } => ("grove", GROVE_TEXT),
            Kind::Backpack { address } => ("backpack", address),
        }
    }

    fn init_grove(&self, i2c: &mut Bus, rgb: Option<u8>) -> Result {
        self.command(i2c, hd44780::CMD_FUNCTION_2LINE)?;
        arduino_hal::delay_ms(5);
        match rgb {
            Some(GROVE_RGB_V5) => {
                i2c.write(GROVE_RGB_V5, &[0x00, 0x07])?; // reset
                arduino_hal::delay_ms(1);
                i2c.write(GROVE_RGB_V5, &[0x04, 0x15])?; // enable R, G, B outputs
            }
            Some(address) => {
                i2c.write(address, &[0x00, 0x00])?;
                i2c.write(address, &[0x08, 0xFF])?;
                i2c.write(address, &[0x01, 0x20])?;
            }
            None => {}
        }
        Ok(())
    }

    /// HD44780 power-on dance to get from 8-bit into 4-bit mode.
    fn init_backpack(&self, i2c: &mut Bus, address: u8) -> Result {
        for wait_us in [4500, 4500, 150] {
            i2c.write(address, &hd44780::pulse(0x30))?;
            arduino_hal::delay_us(wait_us);
        }
        i2c.write(address, &hd44780::pulse(0x20))?;
        arduino_hal::delay_us(150);
        Ok(())
    }

    fn command(&self, i2c: &mut Bus, command: u8) -> Result {
        match self.kind {
            Kind::Grove { .. } => i2c.write(GROVE_TEXT, &[0x80, command]),
            Kind::Backpack { address } => i2c.write(address, &hd44780::byte(command, false)),
        }
    }

    fn data(&self, i2c: &mut Bus, bytes: &[u8]) -> Result {
        match self.kind {
            Kind::Grove { .. } => {
                let mut frame = [0x40; 17];
                frame[1..=bytes.len()].copy_from_slice(bytes);
                i2c.write(GROVE_TEXT, &frame[..=bytes.len()])
            }
            // Each byte the PCF8574 receives sets its pins, so a whole row of
            // E pulses goes out in one transaction. At 100 kHz every pin
            // change lasts ~90 us, longer than the LCD needs per character.
            Kind::Backpack { address } => {
                let mut frame = [0u8; 64];
                let used = hd44780::characters(bytes, &mut frame);
                i2c.write(address, &frame[..used])
            }
        }
    }

    pub fn write_row(&self, i2c: &mut Bus, row: u8, cells: &[u8; 16]) -> Result {
        self.command(i2c, hd44780::row_command(row))?;
        self.data(i2c, cells)
    }

    /// Backlight colour on the Grove RGB; the other LCDs have no colour.
    pub fn set_rgb(&self, i2c: &mut Bus, [r, g, b]: [u8; 3]) -> Result {
        let (address, registers) = match self.kind {
            Kind::Grove { rgb: Some(GROVE_RGB_V5) } => (GROVE_RGB_V5, [0x06, 0x07, 0x08]),
            Kind::Grove { rgb: Some(address) } => (address, [0x04, 0x03, 0x02]),
            _ => return Ok(()),
        };
        for (register, value) in registers.into_iter().zip([r, g, b]) {
            i2c.write(address, &[register, value])?;
        }
        Ok(())
    }
}

fn detect(i2c: &mut Bus) -> Option<Kind> {
    // A one-byte write is harmless to both: on a PCF8574 it just sets pins
    // (backlight on, E low), on the Grove it's an empty command.
    let mut present = |address: u8| i2c.write(address, &[BACKLIGHT]).is_ok();
    if let Some(&address) = BACKPACK_COMMON.iter().find(|&&a| present(a)) {
        return Some(Kind::Backpack { address });
    }
    if present(GROVE_TEXT) {
        let rgb = [GROVE_RGB_V5, GROVE_RGB_V4].into_iter().find(|&a| present(a));
        return Some(Kind::Grove { rgb });
    }
    BACKPACK_OTHER
        .iter()
        .find(|&&a| present(a))
        .map(|&address| Kind::Backpack { address })
}
