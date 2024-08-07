#![no_std]

pub mod spi;
use embedded_hal::{delay::DelayNs, spi::SpiDevice};

pub mod rx;
pub mod tx;

#[cfg(feature = "smoltcp")]
pub mod smoltcp_phy;

/// Max raw frame array size
pub const RAW_FRAME_LENGTH_MAX: usize = 1518;

/// Trait representing PHY layer of ENC424J600
pub trait EthPhy {
    type Error;
    fn recv_packet(&mut self, is_poll: bool) -> Result<rx::RxPacket, Error<Self::Error>>;
    fn send_packet(&mut self, packet: &tx::TxPacket) -> Result<(), Error<Self::Error>>;
}

/// TODO: Improve these error types
#[derive(Debug)]
pub enum Error<E> {
    Spi(E),
    RegisterError,
    // TODO: Better name?
    NoRxPacketError,
}

impl<S: embedded_hal::spi::Error> From<S> for Error<S> {
    fn from(e: S) -> Self {
        Self::Spi(e)
    }
}

/// ENC424J600 controller in SPI mode
pub struct Enc424j600<SPI> {
    spi_port: spi::SpiPort<SPI>,
    rx_buf: rx::RxBuffer,
    tx_buf: tx::TxBuffer,
}

impl<SPI: SpiDevice> Enc424j600<SPI> {
    pub fn new(spi: SPI) -> Self {
        Enc424j600 {
            spi_port: spi::SpiPort::new(spi),
            rx_buf: rx::RxBuffer::new(),
            tx_buf: tx::TxBuffer::new(),
        }
    }

    pub fn init(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<SPI::Error>> {
        self.reset(delay)?;
        self.init_rxbuf()?;
        self.init_txbuf()?;
        Ok(())
    }

    pub fn reset(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<SPI::Error>> {
        // Write 0x1234 to EUDAST
        self.spi_port.write_reg_16b(spi::addrs::EUDAST, 0x1234)?;
        // Verify that EUDAST is 0x1234
        let mut eudast = self.spi_port.read_reg_16b(spi::addrs::EUDAST)?;
        if eudast != 0x1234 {
            return Err(Error::RegisterError);
        }
        // Poll CLKRDY (ESTAT<12>) to check if it is set
        loop {
            let estat = self.spi_port.read_reg_16b(spi::addrs::ESTAT)?;
            if estat & 0x1000 == 0x1000 {
                break;
            }
        }
        // Issue system reset - set ETHRST (ECON2<4>) to 1
        self.spi_port
            .send_opcode(spi::opcodes::OneByteOpcode::SETETHRST)?;
        delay.delay_us(25);
        // Verify that EUDAST is 0x0000
        eudast = self.spi_port.read_reg_16b(spi::addrs::EUDAST)?;
        if eudast != 0x0000 {
            return Err(Error::RegisterError);
        }
        delay.delay_us(256);
        Ok(())
    }

    pub fn init_rxbuf(&mut self) -> Result<(), Error<SPI::Error>> {
        // Set ERXST pointer
        self.spi_port
            .write_reg_16b(spi::addrs::ERXST, self.rx_buf.get_start_addr())?;
        // Set ERXTAIL pointer
        self.spi_port
            .write_reg_16b(spi::addrs::ERXTAIL, self.rx_buf.get_tail_addr())?;
        // Set MAMXFL to maximum number of bytes in each accepted packet
        self.spi_port
            .write_reg_16b(spi::addrs::MAMXFL, RAW_FRAME_LENGTH_MAX as u16)?;
        // Enable RX - set RXEN (ECON1<0>) to 1
        self.spi_port
            .send_opcode(spi::opcodes::OneByteOpcode::ENABLERX)?;
        Ok(())
    }

    pub fn init_txbuf(&mut self) -> Result<(), Error<SPI::Error>> {
        // Set EGPWRPT pointer
        self.spi_port.write_reg_16b(spi::addrs::EGPWRPT, 0x0000)?;
        Ok(())
    }

    /// Set controller to Promiscuous Mode
    pub fn set_promiscuous(&mut self) -> Result<(), Error<SPI::Error>> {
        // From Section 10.12, ENC424J600 Data Sheet:
        // "To accept all incoming frames regardless of content (Promiscuous mode),
        // set the CRCEN, RUNTEN, UCEN, NOTMEEN and MCEN bits."
        let erxfcon_lo = self.spi_port.read_reg_8b(spi::addrs::ERXFCON)?;
        self.spi_port.write_reg_8b(
            spi::addrs::ERXFCON,
            0b0101_1110 | (erxfcon_lo & 0b1010_0001),
        )?;
        Ok(())
    }

    /// Read MAC to [u8; 6]
    pub fn read_mac_addr(&mut self, mac: &mut [u8]) -> Result<(), Error<SPI::Error>> {
        mac[0] = self.spi_port.read_reg_8b(spi::addrs::MAADR1)?;
        mac[1] = self.spi_port.read_reg_8b(spi::addrs::MAADR1 + 1)?;
        mac[2] = self.spi_port.read_reg_8b(spi::addrs::MAADR2)?;
        mac[3] = self.spi_port.read_reg_8b(spi::addrs::MAADR2 + 1)?;
        mac[4] = self.spi_port.read_reg_8b(spi::addrs::MAADR3)?;
        mac[5] = self.spi_port.read_reg_8b(spi::addrs::MAADR3 + 1)?;
        Ok(())
    }

    pub fn write_mac_addr(&mut self, mac: &[u8]) -> Result<(), Error<SPI::Error>> {
        self.spi_port.write_reg_8b(spi::addrs::MAADR1, mac[0])?;
        self.spi_port.write_reg_8b(spi::addrs::MAADR1 + 1, mac[1])?;
        self.spi_port.write_reg_8b(spi::addrs::MAADR2, mac[2])?;
        self.spi_port.write_reg_8b(spi::addrs::MAADR2 + 1, mac[3])?;
        self.spi_port.write_reg_8b(spi::addrs::MAADR3, mac[4])?;
        self.spi_port.write_reg_8b(spi::addrs::MAADR3 + 1, mac[5])?;
        Ok(())
    }
}

impl<SPI: SpiDevice> EthPhy for Enc424j600<SPI> {
    type Error = SPI::Error;

    /// Receive the next packet and return it
    /// Set is_poll to true for returning until PKTIF is set;
    /// Set is_poll to false for returning Err when PKTIF is not set
    fn recv_packet(&mut self, is_poll: bool) -> Result<rx::RxPacket, Error<SPI::Error>> {
        // Poll PKTIF (EIR<4>) to check if it is set
        loop {
            let eir = self.spi_port.read_reg_16b(spi::addrs::EIR)?;
            if eir & 0x40 == 0x40 {
                break;
            }
            if !is_poll {
                return Err(Error::NoRxPacketError);
            }
        }
        // Set ERXRDPT pointer to next_addr
        self.spi_port
            .write_reg_16b(spi::addrs::ERXRDPT, self.rx_buf.get_next_addr())?;
        // Read 2 bytes to update next_addr
        let mut next_addr_buf = [0; 3];
        self.spi_port.read_rxdat(&mut next_addr_buf, 2)?;
        self.rx_buf
            .set_next_addr((next_addr_buf[1] as u16) | ((next_addr_buf[2] as u16) << 8));
        // Read 6 bytes to update rsv
        let mut rsv_buf = [0; 7];
        self.spi_port.read_rxdat(&mut rsv_buf, 6)?;
        // Construct an RxPacket
        // TODO: can we directly assign to fields instead of using functions?
        let mut rx_packet = rx::RxPacket::new();
        // Get and update frame length
        rx_packet.write_to_rsv(&rsv_buf[1..]);
        rx_packet.update_frame_length();
        // Read frame bytes
        let mut frame_buf = [0; RAW_FRAME_LENGTH_MAX];
        self.spi_port
            .read_rxdat(&mut frame_buf, rx_packet.get_frame_length())?;
        rx_packet.copy_frame_from(&frame_buf[1..]);
        // Set ERXTAIL pointer to (next_addr - 2)
        // * Assume head, tail, next and wrap addresses are word-aligned (even)
        // - If next_addr is at least (start_addr+2), then set tail pointer to the word right before next_addr
        if self.rx_buf.get_next_addr() > self.rx_buf.get_start_addr() {
            self.spi_port
                .write_reg_16b(spi::addrs::ERXTAIL, self.rx_buf.get_next_addr() - 2)?;
        // - Otherwise, next_addr will wrap, so set tail pointer to the last word address of RX buffer
        } else {
            self.spi_port
                .write_reg_16b(spi::addrs::ERXTAIL, rx::RX_MAX_ADDRESS - 1)?;
        }
        // Decrement PKTCNT - set PKTDEC (ECON1<8>)
        self.spi_port
            .send_opcode(spi::opcodes::OneByteOpcode::SETPKTDEC)?;
        // Return the RxPacket
        Ok(rx_packet)
    }

    /// Send an established packet
    fn send_packet(&mut self, packet: &tx::TxPacket) -> Result<(), Error<SPI::Error>> {
        // Set EGPWRPT pointer to next_addr
        self.spi_port
            .write_reg_16b(spi::addrs::EGPWRPT, self.tx_buf.get_next_addr())?;
        // Copy packet data to SRAM Buffer
        // 1-byte Opcode is included
        let mut txdat_buf: [u8; RAW_FRAME_LENGTH_MAX + 1] = [0; RAW_FRAME_LENGTH_MAX + 1];
        packet.write_frame_to(&mut txdat_buf[1..]);
        self.spi_port
            .write_txdat(&mut txdat_buf, packet.get_frame_length())?;
        // Set ETXST to packet start address
        self.spi_port
            .write_reg_16b(spi::addrs::ETXST, self.tx_buf.get_next_addr())?;
        // Set ETXLEN to packet length
        self.spi_port
            .write_reg_16b(spi::addrs::ETXLEN, packet.get_frame_length() as u16)?;
        // Send packet - set TXRTS (ECON1<1>) to start transmission
        self.spi_port
            .send_opcode(spi::opcodes::OneByteOpcode::SETTXRTS)?;
        // Poll TXRTS (ECON1<1>) to check if it is reset
        loop {
            let econ1_lo = self.spi_port.read_reg_8b(spi::addrs::ECON1)?;
            if econ1_lo & 0x02 == 0 {
                break;
            }
        }
        // TODO: Read ETXSTAT to understand Ethernet transmission status
        // (See: Register 9-2, ENC424J600 Data Sheet)
        // Update TX buffer start address
        // * Assume TX buffer consumes the entire general-purpose SRAM block
        self.tx_buf.set_next_addr(
            (self.tx_buf.get_next_addr() + packet.get_frame_length() as u16)
                % self.rx_buf.get_start_addr()
                - self.tx_buf.get_start_addr(),
        );
        Ok(())
    }
}
