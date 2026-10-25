//! Motor output. The rest of the firmware only sees `Motors::set`, so a
//! different driver board means a new impl here and nothing else.

use uno_brain::fsm::Drive;

pub trait Motors {
    /// Signed PWM per wheel, -255..255; positive drives forward.
    fn set(&mut self, drive: Drive);
}

/// `--no-default-features` build: the wheels never move.
pub struct NoMotors;

impl Motors for NoMotors {
    fn set(&mut self, _drive: Drive) {}
}

#[cfg(feature = "drv8833")]
pub use drv8833::Drv8833;

#[cfg(feature = "drv8833")]
mod drv8833 {
    use super::Motors;
    use arduino_hal::hal::port::{PB0, PB1, PB2, PD5, PD6};
    use arduino_hal::port::mode::{Output, PwmOutput};
    use arduino_hal::port::Pin;
    use arduino_hal::simple_pwm::{IntoPwmPin, Prescaler, PwmPinOps, Timer0Pwm, Timer1Pwm};
    use uno_brain::fsm::Drive;

    /// DRV8833, same wiring as the R4 WiFi sketch: left IN1/IN2 on D5/D6,
    /// right IN1/IN2 on D9/D10, shared nSLEEP on D8. Prescale64 gives ~980 Hz
    /// fast PWM on both timers.
    pub struct Drv8833 {
        left_in1: Pin<PwmOutput<Timer0Pwm>, PD5>,
        left_in2: Pin<PwmOutput<Timer0Pwm>, PD6>,
        right_in1: Pin<PwmOutput<Timer1Pwm>, PB1>,
        right_in2: Pin<PwmOutput<Timer1Pwm>, PB2>,
        _sleep: Pin<Output, PB0>,
        last: Option<Drive>,
    }

    impl Drv8833 {
        pub fn new(
            tc0: arduino_hal::pac::TC0,
            tc1: arduino_hal::pac::TC1,
            d5: Pin<Output, PD5>,
            d6: Pin<Output, PD6>,
            d9: Pin<Output, PB1>,
            d10: Pin<Output, PB2>,
            mut d8: Pin<Output, PB0>,
        ) -> Self {
            let timer0 = Timer0Pwm::new(tc0, Prescaler::Prescale64);
            let timer1 = Timer1Pwm::new(tc1, Prescaler::Prescale64);
            d8.set_high();
            Drv8833 {
                left_in1: d5.into_pwm(&timer0),
                left_in2: d6.into_pwm(&timer0),
                right_in1: d9.into_pwm(&timer1),
                right_in2: d10.into_pwm(&timer1),
                _sleep: d8,
                last: None,
            }
        }
    }

    /// Forward: IN1 = PWM, IN2 = low. Reverse: the other way round. Zero
    /// disconnects both from the timer so the pins sit low (coast), avoiding
    /// the one-tick glitch fast PWM produces at duty 0.
    fn channel<TC, A: PwmPinOps<TC>, B: PwmPinOps<TC>>(
        in1: &mut Pin<PwmOutput<TC>, A>,
        in2: &mut Pin<PwmOutput<TC>, B>,
        speed: i16,
    ) {
        let duty = speed.unsigned_abs().min(255) as u8;
        if speed > 0 {
            in2.disable();
            in1.set_duty(duty);
            in1.enable();
        } else if speed < 0 {
            in1.disable();
            in2.set_duty(duty);
            in2.enable();
        } else {
            in1.disable();
            in2.disable();
        }
    }

    impl Motors for Drv8833 {
        fn set(&mut self, drive: Drive) {
            if self.last == Some(drive) {
                return;
            }
            self.last = Some(drive);
            channel(&mut self.left_in1, &mut self.left_in2, drive.left);
            channel(&mut self.right_in1, &mut self.right_in2, drive.right);
        }
    }
}
