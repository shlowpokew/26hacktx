//! Everything the Uno does that doesn't touch hardware, kept separate so it
//! can be unit tested on a laptop with `cargo test`.

#![cfg_attr(not(test), no_std)]

pub mod faces;
pub mod fsm;
pub mod hd44780;
pub mod protocol;
pub mod ring;
