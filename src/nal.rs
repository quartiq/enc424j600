use core::cell::RefCell;
use core::convert::TryFrom;
use heapless::{consts, Vec};
use embedded_nal as nal;
use nal::nb;
use smoltcp as net;
use embedded_hal::{
    blocking::spi::Transfer,
    digital::v2::OutputPin
};
pub use embedded_time as time;
use time::duration::*;

#[derive(Debug)]
pub enum NetworkError {
    NoSocket,
    ConnectionFailure,
    ReadFailure,
    WriteFailure,
    Unsupported,
    TimeFault,
}

pub type NetworkInterface<SPI, NSS> = net::iface::EthernetInterface<
    'static,
    crate::smoltcp_phy::SmoltcpDevice<
        crate::SpiEth<SPI, NSS, fn(u32)>
    >,
>;

pub struct NetworkStack<'a, SPI, NSS, IntClock>
where
    SPI: 'static + Transfer<u8>,
    NSS: 'static + OutputPin,
    IntClock: time::Clock<T = u32>,
{
    network_interface: RefCell<NetworkInterface<SPI, NSS>>,
    sockets: RefCell<net::socket::SocketSet<'a>>,
    next_port: RefCell<u16>,
    unused_handles: RefCell<Vec<net::socket::SocketHandle, consts::U16>>,
    time_ms: RefCell<u32>,
    last_update_instant: RefCell<Option<time::Instant<IntClock>>>,
    clock: IntClock
}

impl<'a, SPI, NSS, IntClock> NetworkStack<'a, SPI, NSS, IntClock>
where
    SPI: Transfer<u8>,
    NSS: OutputPin,
    IntClock: time::Clock<T = u32>,
{
    pub fn new(interface: NetworkInterface<SPI, NSS>, sockets: net::socket::SocketSet<'a>, clock: IntClock) -> Self {
        let mut unused_handles: Vec<net::socket::SocketHandle, consts::U16> = Vec::new();
        for socket in sockets.iter() {
            unused_handles.push(socket.handle()).unwrap();
        }
        NetworkStack {
            network_interface: RefCell::new(interface),
            sockets: RefCell::new(sockets),
            next_port: RefCell::new(49152),
            unused_handles: RefCell::new(unused_handles),
            time_ms: RefCell::new(0),
            last_update_instant: RefCell::new(None),
            clock,
        }
    }

    // Include auto_time_update to allow Instant::now() to not be called
    // Instant::now() is not safe to call in `init()` context
    pub fn update(&self, auto_time_update: bool) -> Result<bool, NetworkError> {
        if auto_time_update {
            // Check if it is the first time the stack has updated the time itself
            let now = match *self.last_update_instant.borrow() {
                // If it is the first time, do not advance time
                // Simply store the current instant to initiate time updating
                None => self.clock.try_now().map_err(|_| NetworkError::TimeFault)?,
                // If it was updated before, advance time and update last_update_instant
                Some(instant) => {
                    // Calculate elapsed time
                    let now = self.clock.try_now().map_err(|_| NetworkError::TimeFault)?;
                    let duration = now.checked_duration_since(&instant).ok_or(NetworkError::TimeFault)?;
                    let duration_ms = time::duration::Milliseconds::<u32>::try_from(duration).map_err(|_| NetworkError::TimeFault)?;
                    // Adjust duration into ms (note: decimal point truncated)
                    self.advance_time(*duration_ms.integer());
                    now
                }
            };
            self.last_update_instant.replace(Some(now));
        }
        match self.network_interface.borrow_mut().poll(
            &mut self.sockets.borrow_mut(),
            net::time::Instant::from_millis(*self.time_ms.borrow() as i64),
        ) {
            Ok(changed) => Ok(!changed),
            Err(_e) => {
                Ok(true)
            }
        }
    }

    pub fn advance_time(&self, duration: u32) {
        let time = self.time_ms.try_borrow().unwrap().wrapping_add(duration);
        self.time_ms.replace(time);
    }

    fn get_ephemeral_port(&self) -> u16 {
        // Get the next ephemeral port
        let current_port = self.next_port.borrow().clone();
        let (next, wrap) = self.next_port.borrow().overflowing_add(1);
        *self.next_port.borrow_mut() = if wrap { 49152 } else { next };
        return current_port;
    }
}
impl<'a, SPI, NSS, IntClock> nal::TcpStack for NetworkStack<'a, SPI, NSS, IntClock>
where
    SPI: Transfer<u8>,
    NSS: OutputPin,
    IntClock: time::Clock<T = u32>,
{
    type TcpSocket = net::socket::SocketHandle;
    type Error = NetworkError;
    fn open(&self, _mode: nal::Mode) -> Result<Self::TcpSocket, Self::Error> {
        match self.unused_handles.borrow_mut().pop() {
            Some(handle) => {
                // Abort any active connections on the handle.
                let mut sockets = self.sockets.borrow_mut();
                let internal_socket: &mut net::socket::TcpSocket = &mut *sockets.get(handle);
                internal_socket.abort();
                Ok(handle)
            }
            None => Err(NetworkError::NoSocket),
        }
    }

    // Ideally connect is only to be performed in `init()` of `main.rs`
    // Calling `Instant::now()` of `rtic::cyccnt` would face correctness issue during `init()`
    fn connect(
        &self,
        socket: Self::TcpSocket,
        remote: nal::SocketAddr,
    ) -> Result<Self::TcpSocket, Self::Error> {
        let address = {
            let mut sockets = self.sockets.borrow_mut();
            let internal_socket: &mut net::socket::TcpSocket = &mut *sockets.get(socket);
            // If we're already in the process of connecting, ignore the request silently.
            if internal_socket.is_open() {
                return Ok(socket);
            }
            match remote.ip() {
                nal::IpAddr::V4(addr) => {
                    let address =
                        net::wire::Ipv4Address::from_bytes(&addr.octets()[..]);
                    internal_socket
                        .connect((address, remote.port()), self.get_ephemeral_port())
                        .map_err(|_| NetworkError::ConnectionFailure)?;
                    net::wire::IpAddress::Ipv4(address)
                }
                nal::IpAddr::V6(addr) => {
                    let address = net::wire::Ipv6Address::from_parts(&addr.segments()[..]);
                    internal_socket.connect((address, remote.port()), self.get_ephemeral_port())
                        .map_err(|_| NetworkError::ConnectionFailure)?;
                    net::wire::IpAddress::Ipv6(address)
                }
            }
        };
        // Blocking connect
        loop {
            match self.is_connected(&socket) {
                Ok(true) => break,
                _ => {
                    let mut sockets = self.sockets.borrow_mut();
                    let internal_socket: &mut net::socket::TcpSocket = &mut *sockets.get(socket);
                    // If the connect got ACK->RST, it will end up in Closed TCP state
                    // Perform reconnection in this case
                    if internal_socket.state() == net::socket::TcpState::Closed {
                        internal_socket.close();
                        internal_socket
                            .connect((address, remote.port()), self.get_ephemeral_port())
                            .map_err(|_| NetworkError::ConnectionFailure)?;
                    }
                }
            }
            // Avoid using Instant::now() and Advance time manually
            self.update(false)?;
            {
                self.advance_time(1);
            }
        }
        Ok(socket)
    }

    fn is_connected(&self, socket: &Self::TcpSocket) -> Result<bool, Self::Error> {
        let mut sockets = self.sockets.borrow_mut();
        let socket: &mut net::socket::TcpSocket = &mut *sockets.get(*socket);
        Ok(socket.may_send() && socket.may_recv())
    }

    fn write(&self, socket: &mut Self::TcpSocket, buffer: &[u8]) -> nb::Result<usize, Self::Error> {
        let mut non_queued_bytes = &buffer[..];
        while non_queued_bytes.len() != 0 {
            let result = {
                let mut sockets = self.sockets.borrow_mut();
                let socket: &mut net::socket::TcpSocket = &mut *sockets.get(*socket);
                let result = socket.send_slice(non_queued_bytes);
                result
            };
            match result {
                Ok(num_bytes) => {
                    // In case the buffer is filled up, push bytes into ethernet driver
                    if num_bytes != non_queued_bytes.len() {
                        self.update(true)?;
                    }
                    // Process the unwritten bytes again, if any
                    non_queued_bytes = &non_queued_bytes[num_bytes..]
                }
                Err(_) => return Err(nb::Error::Other(NetworkError::WriteFailure)),
            }
        }
        Ok(buffer.len())
    }

    fn read(
        &self,
        socket: &mut Self::TcpSocket,
        buffer: &mut [u8],
    ) -> nb::Result<usize, Self::Error> {
        // Enqueue received bytes into the TCP socket buffer
        self.update(true)?;
        let mut sockets = self.sockets.borrow_mut();
        let socket: &mut net::socket::TcpSocket = &mut *sockets.get(*socket);
        let result = socket.recv_slice(buffer);
        match result {
            Ok(num_bytes) => Ok(num_bytes),
            Err(_) => Err(nb::Error::Other(NetworkError::ReadFailure)),
        }
    }

    fn close(&self, socket: Self::TcpSocket) -> Result<(), Self::Error> {
        let mut sockets = self.sockets.borrow_mut();
        let internal_socket: &mut net::socket::TcpSocket = &mut *sockets.get(socket);
        internal_socket.close();
        self.unused_handles.borrow_mut().push(socket).unwrap();
        Ok(())
    }
}