use embedded_hal::{
    blocking::spi::Transfer,
    digital::v2::OutputPin,
};

pub mod interfaces {
    use embedded_hal::spi;
    /// Must use SPI mode cpol=0, cpha=0
    pub const SPI_MODE: spi::Mode = spi::Mode {
        polarity: spi::Polarity::IdleLow,
        phase: spi::Phase::CaptureOnFirstTransition,
    };
    /// Max freq = 14 MHz
    pub const SPI_CLOCK_FREQ: u32 = 14_000_000;
}

pub mod opcodes {
    /// 1-byte Instructions
    pub const SETETHRST: u8 = 0b1100_1010;
    pub const SETPKTDEC: u8 = 0b1100_1100;
    pub const SETTXRTS: u8 = 0b1101_0100;
    pub const ENABLERX: u8 = 0b1110_1000;
    /// 3-byte Instructions
    pub const WRXRDPT: u8 = 0b0110_0100;    // 8-bit opcode followed by data
    pub const RRXRDPT: u8 = 0b0110_0110;    // 8-bit opcode followed by data
    pub const WGPWRPT: u8 = 0b0110_1100;    // 8-bit opcode followed by data
    pub const RGPWRPT: u8 = 0b0110_1110;    // 8-bit opcode followed by data
    /// N-byte Instructions
    pub const RCRU: u8 = 0b0010_0000;
    pub const WCRU: u8 = 0b0010_0010;
    pub const RRXDATA: u8 = 0b0010_1100;    // 8-bit opcode followed by data
    pub const WGPDATA: u8 = 0b0010_1010;    // 8-bit opcode followed by data
}

pub mod addrs {
    /// SPI Register Mapping
    /// Note: PSP interface use different address mapping
    // SPI Init Reset Registers
    pub const EUDAST: u8 = 0x16;        // 16-bit data
    pub const ESTAT: u8 = 0x1a;         // 16-bit data
    pub const ECON2: u8 = 0x6e;         // 16-bit data
    //
    pub const ERXFCON: u8 = 0x34;       // 16-bit data
    //
    pub const MAADR3: u8 = 0x60;        // 16-bit data
    pub const MAADR2: u8 = 0x62;        // 16-bit data
    pub const MAADR1: u8 = 0x64;        // 16-bit data
    // RX Registers
    pub const ERXRDPT: u8 = 0x8a;       // 16-bit data
    pub const ERXST: u8 = 0x04;         // 16-bit data
    pub const ERXTAIL: u8 = 0x06;       // 16-bit data
    pub const EIR: u8 = 0x1c;           // 16-bit data
    pub const ECON1: u8 = 0x1e;         // 16-bit data
    pub const MAMXFL: u8 = 0x4a;        // 16-bit data
    // TX Registers
    pub const EGPWRPT: u8 = 0x88;       // 16-bit data
    pub const ETXST: u8 = 0x00;         // 16-bit data
    pub const ETXSTAT: u8 = 0x12;       // 16-bit data
    pub const ETXLEN: u8 = 0x02;        // 16-bit data
}

/// Struct for SPI I/O interface on ENC424J600
/// Note: stm32f4xx_hal::spi's pins include: SCK, MISO, MOSI
pub struct SpiPort<SPI: Transfer<u8>,
                   NSS: OutputPin,
                   F: FnMut(u32) -> ()> {
    spi: SPI,
    nss: NSS,
    delay_ns: F,
}

pub enum Error {
    OpcodeError,
    TransferError
}

#[allow(unused_must_use)]
impl <SPI: Transfer<u8>,
      NSS: OutputPin,
      F: FnMut(u32) -> ()> SpiPort<SPI, NSS, F> {
    // TODO: return as Result()
    pub fn new(spi: SPI, mut nss: NSS, delay_ns: F) -> Self {
        nss.set_high();

        SpiPort {
            spi,
            nss,
            delay_ns,
        }
    }

    pub fn read_reg_8b(&mut self, addr: u8) -> Result<u8, Error> {
        // Using RCRU instruction to read using unbanked (full) address
        let r_data = self.rw_addr_u8(opcodes::RCRU, addr, 0)?;
        Ok(r_data)
    }

    pub fn read_reg_16b(&mut self, lo_addr: u8) -> Result<u16, Error> {
        match lo_addr {
            addrs::ERXRDPT | addrs::EGPWRPT => {
                let mut buf: [u8; 3] = [0; 3];
                self.rw_n(
                    &mut buf, match lo_addr {
                        addrs::ERXRDPT => opcodes::RRXRDPT,
                        addrs::EGPWRPT => opcodes::RGPWRPT,
                        _ => unreachable!()
                    }, 2
                )?;
                Ok((buf[2] as u16) << 8 | buf[1] as u16)
            }
            _ => {
                let r_data_lo = self.read_reg_8b(lo_addr)?;
                let r_data_hi = self.read_reg_8b(lo_addr + 1)?;
                // Combine top and bottom 8-bit to return 16-bit
                Ok(((r_data_hi as u16) << 8) | r_data_lo as u16)
            }
        }
    }

    // Currently requires manual slicing (buf[1..]) for the data read back
    pub fn read_rxdat<'a>(&mut self, buf: &'a mut [u8], data_length: usize)
                         -> Result<(), Error> {
        self.rw_n(buf, opcodes::RRXDATA, data_length)
    }

    // Currently requires actual data to be stored in buf[1..] instead of buf[0..]
    // TODO: Maybe better naming?
    pub fn write_txdat<'a>(&mut self, buf: &'a mut [u8], data_length: usize)
                          -> Result<(), Error> {
        self.rw_n(buf, opcodes::WGPDATA, data_length)
    }

    pub fn write_reg_8b(&mut self, addr: u8, data: u8) -> Result<(), Error> {
        // TODO: addr should be separated from w_data
        // Using WCRU instruction to write using unbanked (full) address
        self.rw_addr_u8(opcodes::WCRU, addr, data)?;
        Ok(())
    }

    pub fn write_reg_16b(&mut self, lo_addr: u8, data: u16) -> Result<(), Error> {
        match lo_addr {
            addrs::ERXRDPT | addrs::EGPWRPT => {
                let mut buf: [u8; 3] = [0; 3];
                buf[1] = data as u8 & 8;
                buf[2] = (data >> 8) as u8 & 8;
                self.rw_n(
                    &mut buf, match lo_addr {
                        addrs::ERXRDPT => opcodes::WRXRDPT,
                        addrs::EGPWRPT => opcodes::WGPWRPT,
                        _ => unreachable!()
                    }, 2
                )
            }
            _ => {
                self.write_reg_8b(lo_addr, (data & 0xff) as u8)?;
                self.write_reg_8b(lo_addr + 1, ((data & 0xff00) >> 8) as u8)?;
                Ok(())
            }
        }
    }

    pub fn send_opcode(&mut self, opcode: u8) -> Result<(), Error> {
        match opcode {
            opcodes::SETETHRST | opcodes::SETPKTDEC |
            opcodes::SETTXRTS | opcodes::ENABLERX => {
                let mut buf: [u8; 1] = [0];
                self.rw_n(&mut buf, opcode, 0)
            }
            _ => Err(Error::OpcodeError)
        }
    }

    pub fn delay_us(&mut self, duration: u32) {
        (self.delay_ns)(duration * 1000)
    }

    // TODO: Generalise transfer functions
    // Completes an SPI transfer for reading or writing 8-bit data,
    // and returns the data read/written.
    // It starts with an 8-bit instruction, followed by an 8-bit address.
    fn rw_addr_u8(&mut self, opcode: u8, addr: u8, data: u8)
                 -> Result<u8, Error> {
        // Enable chip select
        self.nss.set_low();
        // Start writing to SLAVE
        // TODO: don't just use 3 bytes
        let mut buf: [u8; 3] = [0; 3];
        buf[0] = opcode;
        buf[1] = addr;
        buf[2] = data;
        match self.spi.transfer(&mut buf) {
            Ok(_) => {
                // Disable chip select
                (self.delay_ns)(60);
                self.nss.set_high();
                (self.delay_ns)(30);
                Ok(buf[2])
            },
            // TODO: Maybe too naive?
            Err(_) => {
                // Disable chip select
                (self.delay_ns)(60);
                self.nss.set_high();
                (self.delay_ns)(30);
                Err(Error::TransferError)
            }
        }
    }

    // TODO: Generalise transfer functions
    // TODO: Actual data should start from buf[0], not buf[1]
    // Completes an SPI transfer for reading data to the given buffer,
    // or writing data from the buffer.
    // It sends an 8-bit instruction, followed by either
    // receiving or sending n*8-bit data.
    // The slice of buffer provided must begin with the 8-bit instruction.
    // If n = 0, the transfer will only involve sending the instruction.
    fn rw_n<'a>(&mut self, buf: &'a mut [u8], opcode: u8, data_length: usize)
              -> Result<(), Error> {
        assert!(buf.len() > data_length);
        // Enable chip select
        self.nss.set_low();
        // Start writing to SLAVE
        buf[0] = opcode;
        match self.spi.transfer(&mut buf[..data_length+1]) {
            Ok(_) => {
                // Disable chip select
                self.nss.set_high();
                Ok(())
            },
            // TODO: Maybe too naive?
            Err(_) => {
                // Disable chip select
                self.nss.set_high();
                Err(Error::TransferError)
            }
        }
    }
}
