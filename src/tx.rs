use crate::RAW_FRAME_LENGTH_MAX;

/// Struct for TX Buffer on the hardware
/// TODO: Should be a singleton
pub struct TxBuffer {
    start_addr: u16,
    // The following two fields are controlled by firmware
    next_addr: u16,
    tail_addr: u16,
}

impl TxBuffer {
    pub fn new() -> Self {
        TxBuffer {
            start_addr: 0x0000,
            next_addr: 0x0001,
            tail_addr: 0x0000,
        }
    }

    pub fn set_start_addr(&mut self, addr: u16) {
        self.start_addr = addr;
    }
    pub fn get_start_addr(&self) -> u16 {
        self.start_addr
    }

    pub fn set_next_addr(&mut self, addr: u16) {
        self.next_addr = addr;
    }
    pub fn get_next_addr(&self) -> u16 {
        self.next_addr
    }

    pub fn set_tail_addr(&mut self, addr: u16) {
        self.tail_addr = addr;
    }
    pub fn get_tail_addr(&self) -> u16 {
        self.tail_addr
    }
}

/// Struct for TX Packet
/// TODO: Generalise MAC addresses
pub struct TxPacket {
    frame: [u8; RAW_FRAME_LENGTH_MAX],
    frame_length: usize,
}

impl TxPacket {
    pub fn new() -> Self {
        TxPacket {
            frame: [0; RAW_FRAME_LENGTH_MAX],
            frame_length: 0,
        }
    }

    /// Currently, frame data is copied from an external buffer
    pub fn update_frame(&mut self, raw_frame: &[u8], raw_frame_length: usize) {
        self.frame_length = raw_frame_length;
        for i in 0..self.frame_length {
            self.frame[i] = raw_frame[i];
        }
    }
    pub fn write_frame_to(&self, frame: &mut [u8]) {
        for i in 0..self.frame_length {
            frame[i] = self.frame[i];
        }
    }

    pub fn get_frame_length(&self) -> usize {
        self.frame_length
    }

    pub fn get_frame(&self) -> &[u8] {
        &self.frame
    }
    pub fn get_mut_frame(&mut self) -> &mut [u8] {
        &mut self.frame
    }

    /// TODO: Mostly for debugging only?
    pub fn get_frame_byte(&self, i: usize) -> u8 {
        self.frame[i]
    }
}
