//! TCP/UDP socket API
//!
//! Abstraction over smoltcp sockets for the DAQ server.
//! Provides listen/accept/send/recv for TCP and sendto/recvfrom for UDP.

use alloc::vec::Vec;

/// TCP connection state
pub struct TcpConnection {
    pub remote_addr: [u8; 4],
    pub remote_port: u16,
    pub local_port: u16,
    // smoltcp socket handle will go here
}

/// TCP server listener
pub struct TcpListener {
    pub port: u16,
    // smoltcp socket handle
}

impl TcpListener {
    pub fn bind(port: u16) -> Result<Self, &'static str> {
        // TODO: create smoltcp TCP socket, bind to port
        Ok(Self { port })
    }

    pub fn accept(&mut self) -> Option<TcpConnection> {
        // TODO: check for pending connections
        None
    }
}

impl TcpConnection {
    pub fn send(&mut self, data: &[u8]) -> Result<usize, &'static str> {
        // TODO: write to smoltcp socket
        Ok(data.len())
    }

    pub fn recv(&mut self, buf: &mut [u8]) -> Result<usize, &'static str> {
        // TODO: read from smoltcp socket
        Ok(0)
    }

    pub fn close(&mut self) {
        // TODO: close smoltcp socket
    }
}

/// UDP socket
pub struct UdpSocket {
    pub port: u16,
}

impl UdpSocket {
    pub fn bind(port: u16) -> Result<Self, &'static str> {
        Ok(Self { port })
    }

    pub fn sendto(&mut self, data: &[u8], addr: [u8; 4], port: u16) -> Result<usize, &'static str> {
        Ok(data.len())
    }

    pub fn recvfrom(&mut self, buf: &mut [u8]) -> Result<(usize, [u8; 4], u16), &'static str> {
        Ok((0, [0; 4], 0))
    }
}
