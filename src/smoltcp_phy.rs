use crate::{
    EthPhy, tx, RAW_FRAME_LENGTH_MAX
};
use core::cell;
use smoltcp::{
    phy::{Device, DeviceCapabilities, RxToken, TxToken},
    time::Instant,
    Error
};

pub struct SmoltcpDevice<E: EthPhy> {
    pub eth_phy: cell::RefCell<E>,
    rx_packet_buf: [u8; RAW_FRAME_LENGTH_MAX],
    tx_packet_buf: [u8; RAW_FRAME_LENGTH_MAX]
}

impl<E: EthPhy> SmoltcpDevice<E> {
    pub fn new(eth_phy: E) -> Self {
        SmoltcpDevice {
            eth_phy: cell::RefCell::new(eth_phy),
            rx_packet_buf: [0; RAW_FRAME_LENGTH_MAX],
            tx_packet_buf: [0; RAW_FRAME_LENGTH_MAX]
        }
    }
}

impl<'a, E: 'a + EthPhy> Device<'a> for SmoltcpDevice<E> {
    type RxToken = EthRxToken<'a>;
    type TxToken = EthTxToken<'a, E>;

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = RAW_FRAME_LENGTH_MAX;
        caps
    }

    fn receive(&'a mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        let self_p = (&mut *self) as *mut SmoltcpDevice<E>;
        match self.eth_phy.borrow_mut().recv_packet(false) {
            Ok(rx_packet) => {
                // Write received packet to RX packet buffer
                rx_packet.write_frame_to(&mut self.rx_packet_buf);
                // Construct a RxToken
                let rx_token = EthRxToken {
                    buf: &mut self.rx_packet_buf,
                    len: rx_packet.get_frame_length()
                };
                // Construct a blank TxToken
                let tx_token = EthTxToken {
                    buf: &mut self.tx_packet_buf,
                    dev: self_p
                };
                Some((rx_token, tx_token))
            },
            Err(_) => None
        }
    }

    fn transmit(&'a mut self) -> Option<Self::TxToken> {
        let self_p = (&mut *self) as *mut SmoltcpDevice<E>;
        // Construct a blank TxToken
        let tx_token = EthTxToken {
            buf: &mut self.tx_packet_buf,
            dev: self_p
        };
        Some(tx_token)
    }
}

pub struct EthRxToken<'a> {
    buf: &'a mut [u8],
    len: usize
}

impl<'a> RxToken for EthRxToken<'a> {
    fn consume<R, F>(self, _timestamp: Instant, f: F) -> Result<R, Error>
    where
        F: FnOnce(&mut [u8]) -> Result<R, Error>,
    {
        f(&mut self.buf[..self.len])
    }
}

pub struct EthTxToken<'a, E: EthPhy> {
    buf: &'a mut [u8],
    dev: *mut SmoltcpDevice<E>
}

impl<'a, E: 'a + EthPhy> TxToken for EthTxToken<'a, E> {
    fn consume<R, F>(self, _timestamp: Instant, len: usize, f: F) -> Result<R, Error>
    where
        F: FnOnce(&mut [u8]) -> Result<R, Error>,
    {
        let result = f(&mut self.buf[..len]);
        // Construct a TxPacket
        let mut tx_packet = tx::TxPacket::new();
        // Update frame length and write frame bytes
        tx_packet.update_frame(&mut self.buf[..len], len);
        // Send the packet as raw
        let eth_phy = unsafe {
            &mut (*self.dev).eth_phy
        };
        match eth_phy.borrow_mut().send_packet(&tx_packet) {
            Ok(_) => { result },
            Err(_) => Err(Error::Exhausted)
        }
    }
}
