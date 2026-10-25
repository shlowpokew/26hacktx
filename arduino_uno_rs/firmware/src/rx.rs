//! Interrupt-driven USART receive. The hardware only buffers two bytes, and an
//! LCD update over I2C takes a few milliseconds, so bytes from the Pi land in
//! a ring buffer from the RX interrupt instead of being polled.

use avr_device::interrupt::Mutex;
use core::cell::RefCell;
use uno_brain::ring::Ring;

static RING: Mutex<RefCell<Ring<64>>> = Mutex::new(RefCell::new(Ring::new()));

#[avr_device::interrupt(atmega328p)]
fn USART_RX() {
    // SAFETY: only this handler reads UDR0; main only writes through the Usart.
    let dp = unsafe { arduino_hal::Peripherals::steal() };
    let byte = dp.USART0.udr0().read().bits();
    avr_device::interrupt::free(|cs| {
        RING.borrow(cs).borrow_mut().push(byte);
    })
}

pub fn pop() -> Option<u8> {
    avr_device::interrupt::free(|cs| RING.borrow(cs).borrow_mut().pop())
}
