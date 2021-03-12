use core::cell::RefCell;
use core::convert::TryInto;
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
    clock: IntClock,
    connection_timeout_ms: u32,
}

impl<'a, SPI, NSS, IntClock> NetworkStack<'a, SPI, NSS, IntClock>
where
    SPI: Transfer<u8>,
    NSS: OutputPin,
    IntClock: time::Clock<T = u32>,
{
    pub fn new(
        interface: NetworkInterface<SPI, NSS>,
        sockets: net::socket::SocketSet<'a>,
        clock: IntClock,
        connection_timeout_ms: u32,
    ) -> Self {
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
            connection_timeout_ms,
        }
    }

    // Initiate or advance the timer, and return the duration in ms as u32.
    fn update(&self) -> Result<u32, NetworkError> {
        let mut duration_ms: u32 = 0;
        // Check if it is the first time the stack has updated the time itself
        let now = match *self.last_update_instant.borrow() {
            // If it is the first time, do not advance time
            // Simply store the current instant to initiate time updating
            None => self.clock.try_now().map_err(|_| NetworkError::TimeFault)?,
            // If it was updated before, advance time and update last_update_instant
            Some(instant) => {
                // Calculate elapsed time
                let now = self.clock.try_now().map_err(|_| NetworkError::TimeFault)?;
                let mut duration = now.checked_duration_since(&instant);
                // Normally, the wrapping clock should produce a valid duration.
                // However, if `now` is earlier than `instant` (e.g. because the main 
                // application cannot get a valid epoch time during initialisation, 
                // we should still produce a duration that is just 1ms.
                if duration.is_none() {
                    self.time_ms.replace(0);
                    duration = Some(Milliseconds(1_u32)
                        .to_generic::<u32>(IntClock::SCALING_FACTOR)
                        .map_err(|_| NetworkError::TimeFault)?);
                }
                let duration_ms_time: Milliseconds<u32> = duration.unwrap().try_into()
                    .map_err(|_| NetworkError::TimeFault)?;
                duration_ms = *duration_ms_time.integer();
                // Adjust duration into ms (note: decimal point truncated)
                self.advance_time(duration_ms);
                now
            }
        };
        self.last_update_instant.replace(Some(now));
        Ok(duration_ms)
    }

    fn advance_time(&self, duration_ms: u32) {
        let time = self.time_ms.borrow().wrapping_add(duration_ms);
        self.time_ms.replace(time);
    }

    // Poll on the smoltcp interface
    fn poll(&self) -> Result<bool, NetworkError> {
        match self.network_interface.borrow_mut().poll(
            &mut self.sockets.borrow_mut(),
            net::time::Instant::from_millis(*self.time_ms.borrow() as u32),
        ) {
            Ok(changed) => Ok(!changed),
            Err(_e) => {
                Ok(true)
            }
        }
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
                let socket: &mut net::socket::TcpSocket = &mut *sockets.get(handle);
                socket.abort();
                Ok(handle)
            }
            None => Err(NetworkError::NoSocket),
        }
    }

    fn connect(
        &self,
        handle: Self::TcpSocket,
        remote: nal::SocketAddr,
    ) -> Result<Self::TcpSocket, Self::Error> {
        {
            // If the socket has already been connected, ignore the connection
            // request silently.
            let mut sockets = self.sockets.borrow_mut();
            let socket: &mut net::socket::TcpSocket = &mut *sockets.get(handle);
            if socket.state() == net::socket::TcpState::Established {
                return Ok(handle)
            }
        }

        {
            let mut sockets = self.sockets.borrow_mut();
            let socket: &mut net::socket::TcpSocket = &mut *sockets.get(handle);
            // abort() instead of close() prevents TcpSocket::connect() from
            // raising an error
            socket.abort();
            match remote.ip() {
                nal::IpAddr::V4(addr) => {
                    let address =
                        net::wire::Ipv4Address::from_bytes(&addr.octets()[..]);
                    socket
                        .connect((address, remote.port()), self.get_ephemeral_port())
                        .map_err(|_| NetworkError::ConnectionFailure)?;
                    net::wire::IpAddress::Ipv4(address)
                },
                nal::IpAddr::V6(addr) => {
                    let address =
                        net::wire::Ipv6Address::from_parts(&addr.segments()[..]);
                    socket
                        .connect((address, remote.port()), self.get_ephemeral_port())
                        .map_err(|_| NetworkError::ConnectionFailure)?;
                    net::wire::IpAddress::Ipv6(address)
                }
            }
        };

        // Loop to wait until the socket is staying established or closed,
        // or the connection attempt has timed out.
        let mut timeout_ms: u32 = 0;
        loop {
            {
                let mut sockets = self.sockets.borrow_mut();
                let socket: &mut net::socket::TcpSocket = &mut *sockets.get(handle);
                // TCP state at ESTABLISHED means there is connection, so
                // simply return the socket.
                if socket.state() == net::socket::TcpState::Established {
                    return Ok(handle)
                }
                // TCP state at CLOSED implies that the remote rejected connection;
                // In this case, abort the connection, and then return the socket
                // for re-connection in the future.
                if socket.state() == net::socket::TcpState::Closed {
                    socket.abort();
                    // TODO: Return Err(), but would require changes in quartiq/minimq
                    return Ok(handle)
                }
            }
            // Any TCP states other than CLOSED and ESTABLISHED are considered
            // "transient", so this function should keep waiting and let smoltcp poll
            // (e.g. for handling echo reqeust/reply) at the same time.
            timeout_ms += self.update()?;
            self.poll()?;
            // Time out, and return the socket for re-connection in the future.
            if timeout_ms > self.connection_timeout_ms {
                // TODO: Return Err(), but would require changes in quartiq/minimq
                return Ok(handle)
            }
        }
    }

    fn is_connected(&self, handle: &Self::TcpSocket) -> Result<bool, Self::Error> {
        let mut sockets = self.sockets.borrow_mut();
        let socket: &mut net::socket::TcpSocket = &mut *sockets.get(*handle);
        Ok(socket.state() == net::socket::TcpState::Established)
    }

    fn write(&self, handle: &mut Self::TcpSocket, buffer: &[u8]) -> nb::Result<usize, Self::Error> {
        let mut write_error = false;
        let mut non_queued_bytes = &buffer[..];
        while non_queued_bytes.len() != 0 {
            let result = {
                let mut sockets = self.sockets.borrow_mut();
                let socket: &mut net::socket::TcpSocket = &mut *sockets.get(*handle);
                let result = socket.send_slice(non_queued_bytes);
                result
            };
            match result {
                Ok(num_bytes) => {
                    // In case the buffer is filled up, push bytes into ethernet driver
                    if num_bytes != non_queued_bytes.len() {
                        self.update()?;
                        self.poll()?;
                    }
                    // Process the unwritten bytes again, if any
                    non_queued_bytes = &non_queued_bytes[num_bytes..]
                }
                Err(_) => {
                    write_error = true;
                    break;
                }
            }
        }
        if write_error {
            // Close the socket to push it back to the array, for
            // re-opening the socket in the future
            self.close(*handle)?;
            return Err(nb::Error::Other(NetworkError::WriteFailure))
        }   
        Ok(buffer.len())
    }

    fn read(
        &self,
        handle: &mut Self::TcpSocket,
        buffer: &mut [u8],
    ) -> nb::Result<usize, Self::Error> {
        // Enqueue received bytes into the TCP socket buffer
        self.update()?;
        self.poll()?;
        {
            let mut sockets = self.sockets.borrow_mut();
            let socket: &mut net::socket::TcpSocket = &mut *sockets.get(*handle);
            let result = socket.recv_slice(buffer);
            match result {
                Ok(num_bytes) => { return Ok(num_bytes) },
                Err(_) => {},
            }
        }
        // Close the socket to push it back to the array, for
        // re-opening the socket in the future
        self.close(*handle)?;
        Err(nb::Error::Other(NetworkError::ReadFailure))
    }

    fn close(&self, handle: Self::TcpSocket) -> Result<(), Self::Error> {
        let mut sockets = self.sockets.borrow_mut();
        let socket: &mut net::socket::TcpSocket = &mut *sockets.get(handle);
        socket.close();
        let mut unused_handles = self.unused_handles.borrow_mut();
        if unused_handles.iter().find(|&x| *x == handle).is_none() {
            unused_handles.push(handle).unwrap();
        }
        Ok(())
    }
}