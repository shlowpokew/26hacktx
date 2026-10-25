//! Quadrature wheel encoders. Each motor's yellow wire (A) is on an external
//! interrupt pin and its green wire (B) on a plain input:
//!
//! - left:  A on D2 (INT0), B on D4
//! - right: A on D3 (INT1), B on D7
//!
//! Every edge of A fires an interrupt, and B tells which way the wheel is
//! turning: two counts per encoder cycle. All four pins are on PORTD, so one
//! read of PIND gives both channels.

use arduino_hal::hal::port::{PD2, PD3, PD4, PD7};
use arduino_hal::port::mode::{Input, PullUp};
use arduino_hal::port::Pin;
use avr_device::interrupt::Mutex;
use core::cell::Cell;

static LEFT: Mutex<Cell<i32>> = Mutex::new(Cell::new(0));
static RIGHT: Mutex<Cell<i32>> = Mutex::new(Cell::new(0));

const LEFT_A: u8 = 1 << 2;
const LEFT_B: u8 = 1 << 4;
const RIGHT_A: u8 = 1 << 3;
const RIGHT_B: u8 = 1 << 7;

/// Takes the pins so nothing else can reconfigure them.
pub fn init(
    exint: arduino_hal::pac::EXINT,
    _left_a: Pin<Input<PullUp>, PD2>,
    _left_b: Pin<Input<PullUp>, PD4>,
    _right_a: Pin<Input<PullUp>, PD3>,
    _right_b: Pin<Input<PullUp>, PD7>,
) {
    // ISC00 and ISC10: interrupt on any logical change of INT0 and INT1.
    // SAFETY: only the ISC0/ISC1 bits are set; the rest are reserved zeros.
    exint.eicra().write(|w| unsafe { w.bits(0b0101) });
    exint.eifr().write(|w| unsafe { w.bits(0b11) });
    exint.eimsk().write(|w| unsafe { w.bits(0b11) });
}

fn pind() -> u8 {
    // SAFETY: a read-only look at the input register.
    let dp = unsafe { arduino_hal::Peripherals::steal() };
    dp.PORTD.pind().read().bits()
}

/// A and B differ right after an A edge when the wheel turns one way and
/// match when it turns the other. Which way is "forward" is up to the Pi
/// (`LEFT_ENCODER_SIGN` / `RIGHT_ENCODER_SIGN`).
fn step(count: &Mutex<Cell<i32>>, a: bool, b: bool) {
    avr_device::interrupt::free(|cs| {
        let count = count.borrow(cs);
        count.set(count.get().wrapping_add(if a != b { 1 } else { -1 }));
    })
}

#[avr_device::interrupt(atmega328p)]
fn INT0() {
    let pins = pind();
    step(&LEFT, pins & LEFT_A != 0, pins & LEFT_B != 0);
}

#[avr_device::interrupt(atmega328p)]
fn INT1() {
    let pins = pind();
    step(&RIGHT, pins & RIGHT_A != 0, pins & RIGHT_B != 0);
}

/// Running (left, right) counts since boot, read together.
pub fn counts() -> (i32, i32) {
    avr_device::interrupt::free(|cs| (LEFT.borrow(cs).get(), RIGHT.borrow(cs).get()))
}
