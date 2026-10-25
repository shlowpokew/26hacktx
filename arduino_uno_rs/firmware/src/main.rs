//! Arduino Uno firmware: wheel PWM, mission state and LiDAR sectors in from
//! the Pi over USB serial; DRV8833 motor PWM out; kaomoji on a 16x2 I2C LCD
//! (A4 SDA, A5 SCL).
//!
//! All decisions live in `uno_brain`; this file just wires it to pins.

#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

mod clock;
mod lcd;
mod motors;
mod rx;

use arduino_hal::prelude::*;
use arduino_hal::hal::usart::Event;
use arduino_hal::hal::wdt;
use motors::Motors;
use panic_halt as _;
use uno_brain::faces::{self, Screen};
use uno_brain::fsm::{Brain, Drive};
use uno_brain::protocol::Parser;

const CONTROL_MS: u32 = 20;
const DISPLAY_MS: u32 = 100;
const REPORT_MS: u32 = 200;
const LCD_RETRY_MS: u32 = 2000;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // Anything that wedges the loop (a stuck I2C bus, say) resets the board,
    // and a reset leaves the motor pins as inputs: stopped.
    let mut watchdog = wdt::Wdt::new(dp.WDT, &dp.CPU.mcusr());
    watchdog.start(wdt::Timeout::Ms1000).unwrap();

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    serial.listen(Event::RxComplete);

    let mut i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        100_000,
    );

    clock::init(dp.TC2);

    #[cfg(feature = "drv8833")]
    let mut motors = motors::Drv8833::new(
        dp.TC0,
        dp.TC1,
        pins.d5.into_output(),
        pins.d6.into_output(),
        pins.d9.into_output(),
        pins.d10.into_output(),
        pins.d8.into_output(),
    );
    #[cfg(not(feature = "drv8833"))]
    let mut motors = motors::NoMotors;
    let motor_kind = if cfg!(feature = "drv8833") { "drv8833" } else { "none" };

    // SAFETY: all interrupt handlers and the state they share are set up.
    unsafe { avr_device::interrupt::enable() };

    ufmt::uwriteln!(&mut serial, "uno-brain up motors={}\r", motor_kind).unwrap_infallible();

    let mut parser = Parser::new();
    let mut brain = Brain::new(clock::millis());
    let mut output = Drive::STOP;
    let mut lcd: Option<lcd::Lcd> = None;
    let mut shown: Option<Screen> = None;
    let mut next_control = 0u32;
    let mut next_display = 0u32;
    let mut next_report = 0u32;
    let mut next_lcd_attempt = 0u32;

    loop {
        watchdog.feed();

        while let Some(byte) = rx::pop() {
            if let Some(command) = parser.push(byte) {
                brain.on_command(clock::millis(), command);
            }
        }

        let now = clock::millis();

        if due(now, next_control) {
            next_control = now.wrapping_add(CONTROL_MS);
            output = brain.tick(now);
            motors.set(output);
        }

        if due(now, next_display) {
            next_display = now.wrapping_add(DISPLAY_MS);
            if lcd.is_none() && due(now, next_lcd_attempt) {
                next_lcd_attempt = now.wrapping_add(LCD_RETRY_MS);
                lcd = lcd::Lcd::init(&mut i2c, &faces::GLYPHS).ok();
                shown = None;
                if let Some(display) = &lcd {
                    let (kind, address) = display.describe();
                    ufmt::uwriteln!(&mut serial, "lcd={} addr={}\r", kind, address)
                        .unwrap_infallible();
                }
            }
            if let Some(display) = &lcd {
                let screen = faces::screen(&brain, now);
                if draw(display, &mut i2c, &screen, shown.as_ref()).is_ok() {
                    shown = Some(screen);
                } else {
                    lcd = None;
                }
            }
        }

        if due(now, next_report) {
            next_report = now.wrapping_add(REPORT_MS);
            ufmt::uwriteln!(
                &mut serial,
                "st={} mission={} l={} r={} ahead={} near={} bad={}\r",
                brain.state().name(),
                brain.mission().name(),
                output.left,
                output.right,
                brain.ahead_cm(),
                brain.closest_cm(),
                parser.bad_frames,
            )
            .unwrap_infallible();
        }
    }
}

/// Redraw only what changed since the last successful draw.
fn draw(
    display: &lcd::Lcd,
    i2c: &mut arduino_hal::I2c,
    screen: &Screen,
    shown: Option<&Screen>,
) -> Result<(), arduino_hal::i2c::Error> {
    if shown.map(|s| s.top) != Some(screen.top) {
        display.write_row(i2c, 0, &screen.top)?;
    }
    if shown.map(|s| s.bottom) != Some(screen.bottom) {
        display.write_row(i2c, 1, &screen.bottom)?;
    }
    if shown.map(|s| s.rgb) != Some(screen.rgb) {
        display.set_rgb(i2c, screen.rgb)?;
    }
    Ok(())
}

/// Wrap-safe "has `at` arrived".
fn due(now: u32, at: u32) -> bool {
    now.wrapping_sub(at) < u32::MAX / 2
}
