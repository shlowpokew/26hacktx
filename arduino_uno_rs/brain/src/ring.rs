//! Fixed-size byte FIFO for the serial RX interrupt. When it's full new bytes
//! are dropped; the frame CRC catches the gap.

pub struct Ring<const N: usize> {
    buf: [u8; N],
    head: usize,
    len: usize,
}

impl<const N: usize> Ring<N> {
    pub const fn new() -> Self {
        Ring { buf: [0; N], head: 0, len: 0 }
    }

    /// Returns false if the byte was dropped because the ring is full.
    pub fn push(&mut self, byte: u8) -> bool {
        if self.len == N {
            return false;
        }
        self.buf[(self.head + self.len) % N] = byte;
        self.len += 1;
        true
    }

    pub fn pop(&mut self) -> Option<u8> {
        if self.len == 0 {
            return None;
        }
        let byte = self.buf[self.head];
        self.head = (self.head + 1) % N;
        self.len -= 1;
        Some(byte)
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl<const N: usize> Default for Ring<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_in_first_out() {
        let mut ring = Ring::<4>::new();
        assert!(ring.push(1) && ring.push(2) && ring.push(3));
        assert_eq!(ring.pop(), Some(1));
        assert_eq!(ring.pop(), Some(2));
        assert_eq!(ring.pop(), Some(3));
        assert_eq!(ring.pop(), None);
    }

    #[test]
    fn drops_when_full_and_keeps_the_old_bytes() {
        let mut ring = Ring::<3>::new();
        assert!(ring.push(1) && ring.push(2) && ring.push(3));
        assert!(!ring.push(4));
        assert_eq!(ring.len(), 3);
        assert_eq!((ring.pop(), ring.pop(), ring.pop()), (Some(1), Some(2), Some(3)));
        assert!(ring.is_empty());
    }

    #[test]
    fn wraps_around_many_times() {
        let mut ring = Ring::<5>::new();
        let mut expected = 0u8;
        for byte in 0..=255u8 {
            assert!(ring.push(byte));
            if ring.len() == 3 {
                assert_eq!(ring.pop(), Some(expected));
                expected = expected.wrapping_add(1);
            }
        }
        while let Some(byte) = ring.pop() {
            assert_eq!(byte, expected);
            expected = expected.wrapping_add(1);
        }
        assert_eq!(expected, 0, "saw all 256 bytes in order");
    }
}
