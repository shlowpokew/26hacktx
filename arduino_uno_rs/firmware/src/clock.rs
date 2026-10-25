//! `millis()` on TC2. TC0 and TC1 are left free for motor PWM (D5/D6, D9/D10).

use avr_device::interrupt::Mutex;
use core::cell::Cell;

static MILLIS: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

/// 16 MHz / 64 prescaler / 250 counts = one compare match per millisecond.
pub fn init(tc2: arduino_hal::pac::TC2) {
    tc2.tccr2a().write(|w| w.wgm2().ctc());
    tc2.ocr2a().write(|w| w.set(249));
    tc2.tccr2b().write(|w| w.cs2().prescale_64());
    tc2.timsk2().write(|w| w.ocie2a().set_bit());
}

#[avr_device::interrupt(atmega328p)]
fn TIMER2_COMPA() {
    avr_device::interrupt::free(|cs| {
        let millis = MILLIS.borrow(cs);
        millis.set(millis.get().wrapping_add(1));
    })
}

pub fn millis() -> u32 {
    avr_device::interrupt::free(|cs| MILLIS.borrow(cs).get())
}
