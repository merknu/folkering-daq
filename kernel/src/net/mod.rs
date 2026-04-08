//! Network stack — smoltcp over Cadence GEM (RP1 Ethernet)
//!
//! Integrates:
//! - Cadence GEM MAC driver (phy::Device trait)
//! - DHCP client for IP address
//! - TCP server on port 7420 (openDAQ Native Streaming)
//! - UDP on port 5353 (mDNS service discovery)

pub mod socket;
pub mod mdns;
pub mod websocket;

use smoltcp::iface::{Config, Interface, SocketHandle, SocketSet};
use smoltcp::phy;
use smoltcp::socket::{dhcpv4, tcp, udp};
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, HardwareAddress, IpAddress, IpCidr, Ipv4Address};
use core::sync::atomic::{AtomicBool, Ordering};

static NET_READY: AtomicBool = AtomicBool::new(false);
static mut OUR_IP: [u8; 4] = [0; 4];

// === smoltcp phy::Device adapter for Cadence GEM ===

pub struct GemDevice;

// RxToken: holds the index of the received packet in the GEM ring
pub struct GemRxToken(usize);

// TxToken: just a marker, actual TX happens in consume()
pub struct GemTxToken;

impl phy::Device for GemDevice {
    type RxToken<'a> = GemRxToken;
    type TxToken<'a> = GemTxToken;

    fn receive(&mut self, _timestamp: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        match crate::drivers::gem::receive() {
            Some((_data, idx)) => Some((GemRxToken(idx), GemTxToken)),
            None => None,
        }
    }

    fn transmit(&mut self, _timestamp: Instant) -> Option<Self::TxToken<'_>> {
        // Always ready if TX ring has space
        Some(GemTxToken)
    }

    fn capabilities(&self) -> phy::DeviceCapabilities {
        let mut caps = phy::DeviceCapabilities::default();
        caps.medium = phy::Medium::Ethernet;
        caps.max_transmission_unit = 1514; // Standard Ethernet MTU
        caps.max_burst_size = Some(1);
        caps
    }
}

impl phy::RxToken for GemRxToken {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        // Get the packet data from GEM ring
        let result = if let Some((data, _idx)) = crate::drivers::gem::receive() {
            let r = f(data);
            crate::drivers::gem::release_rx(self.0);
            r
        } else {
            f(&[])
        };
        result
    }
}

impl phy::TxToken for GemTxToken {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let mut buf = [0u8; 2048];
        let actual_len = len.min(buf.len());
        let result = f(&mut buf[..actual_len]);
        // Send via GEM
        let _ = crate::drivers::gem::transmit(&buf[..actual_len]);
        result
    }
}

// === Network State ===

// Static socket storage (no heap needed for socket metadata)
const TCP_RX_SIZE: usize = 16384;
const TCP_TX_SIZE: usize = 16384;
const UDP_RX_SIZE: usize = 2048;
const UDP_TX_SIZE: usize = 2048;

static mut TCP_RX_BUF: [u8; TCP_RX_SIZE] = [0; TCP_RX_SIZE];
static mut TCP_TX_BUF: [u8; TCP_TX_SIZE] = [0; TCP_TX_SIZE];
static mut UDP_RX_META: [udp::PacketMetadata; 8] = [udp::PacketMetadata::EMPTY; 8];
static mut UDP_TX_META: [udp::PacketMetadata; 8] = [udp::PacketMetadata::EMPTY; 8];
static mut UDP_RX_BUF: [u8; UDP_RX_SIZE] = [0; UDP_RX_SIZE];
static mut UDP_TX_BUF: [u8; UDP_TX_SIZE] = [0; UDP_TX_SIZE];

static mut DEVICE: Option<GemDevice> = None;
static mut IFACE: Option<Interface> = None;
static mut TCP_HANDLE: Option<SocketHandle> = None;
static mut UDP_HANDLE: Option<SocketHandle> = None;

// Socket set storage
static mut SOCKET_SET_BUF: [smoltcp::iface::SocketStorage<'static>; 4] =
    [smoltcp::iface::SocketStorage::EMPTY; 4];
static mut SOCKETS: Option<SocketSet<'static>> = None;

// WebSocket handshake state
static mut WS_UPGRADED: bool = false;

fn now() -> Instant {
    Instant::from_millis(crate::arch::aarch64::timer::millis() as i64)
}

/// Initialize network stack
pub fn init() {
    let rp1 = match crate::drivers::pci::rp1() {
        Some(r) => r,
        None => {
            crate::kprintln!("  NET: RP1 not found");
            return;
        }
    };

    // Initialize Cadence GEM Ethernet MAC
    crate::drivers::gem::init(rp1.eth_base);

    let mac = crate::drivers::gem::mac_address();
    let hw_addr = HardwareAddress::Ethernet(EthernetAddress(mac));

    crate::kprintln!("  NET: MAC {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    unsafe {
        DEVICE = Some(GemDevice);

        // Create smoltcp Interface
        let mut config = Config::new(hw_addr);
        config.random_seed = crate::arch::aarch64::counter_value();

        let device = DEVICE.as_mut().unwrap();
        let mut iface = Interface::new(config, device, now());

        // Static IP for now (DHCP can be added later)
        iface.update_ip_addrs(|addrs| {
            let _ = addrs.push(IpCidr::new(IpAddress::v4(192, 168, 1, 100), 24));
        });
        OUR_IP = [192, 168, 1, 100];

        // Create socket set
        let mut sockets = SocketSet::new(&mut SOCKET_SET_BUF[..]);

        // TCP socket for openDAQ (port 7420)
        let tcp_rx = tcp::SocketBuffer::new(&mut TCP_RX_BUF[..]);
        let tcp_tx = tcp::SocketBuffer::new(&mut TCP_TX_BUF[..]);
        let mut tcp_socket = tcp::Socket::new(tcp_rx, tcp_tx);
        tcp_socket.listen(7420).unwrap();
        let tcp_h = sockets.add(tcp_socket);
        TCP_HANDLE = Some(tcp_h);

        // UDP socket for mDNS (port 5353)
        let udp_rx = udp::PacketBuffer::new(&mut UDP_RX_META[..], &mut UDP_RX_BUF[..]);
        let udp_tx = udp::PacketBuffer::new(&mut UDP_TX_META[..], &mut UDP_TX_BUF[..]);
        let mut udp_socket = udp::Socket::new(udp_rx, udp_tx);
        udp_socket.bind(5353).unwrap();
        let udp_h = sockets.add(udp_socket);
        UDP_HANDLE = Some(udp_h);

        IFACE = Some(iface);
        SOCKETS = Some(sockets);
        WS_UPGRADED = false;
    }

    NET_READY.store(true, Ordering::SeqCst);
    crate::kprintln!("  NET: stack ready, TCP :7420 + UDP :5353");
}

/// Poll network — call from main loop
pub fn poll() {
    if !NET_READY.load(Ordering::Relaxed) { return; }

    unsafe {
        let device = DEVICE.as_mut().unwrap();
        let iface = IFACE.as_mut().unwrap();
        let sockets = SOCKETS.as_mut().unwrap();

        // Drive the smoltcp state machine
        let _ = iface.poll(now(), device, sockets);

        // Handle mDNS queries
        handle_mdns(sockets);

        // Handle openDAQ TCP connections
        handle_opendaq_tcp(sockets);
    }
}

/// Handle mDNS queries on UDP port 5353
unsafe fn handle_mdns(sockets: &mut SocketSet) {
    let udp_h = match UDP_HANDLE { Some(h) => h, None => return };
    let socket = sockets.get_mut::<udp::Socket>(udp_h);

    if !socket.can_recv() { return; }

    if let Ok((data, endpoint)) = socket.recv() {
        if mdns::is_query_for_us(data) {
            let mut response_buf = [0u8; 512];
            let len = mdns::build_response(OUR_IP, &mut response_buf);

            if len > 0 && socket.can_send() {
                let dest = smoltcp::wire::IpEndpoint::new(
                    IpAddress::v4(224, 0, 0, 251), 5353,
                );
                let _ = socket.send_slice(&response_buf[..len], dest);
            }
        }
    }
}

/// Handle openDAQ TCP connections on port 7420
unsafe fn handle_opendaq_tcp(sockets: &mut SocketSet) {
    let tcp_h = match TCP_HANDLE { Some(h) => h, None => return };
    let socket = sockets.get_mut::<tcp::Socket>(tcp_h);

    if !socket.is_active() {
        // Re-listen if connection was closed
        WS_UPGRADED = false;
        let _ = socket.listen(7420);
        return;
    }

    if !socket.may_recv() { return; }

    // Read incoming data
    let mut recv_buf = [0u8; 4096];
    let n = match socket.recv_slice(&mut recv_buf) {
        Ok(n) => n,
        Err(_) => return,
    };
    if n == 0 { return; }

    let data = &recv_buf[..n];

    if !WS_UPGRADED {
        // Check for WebSocket upgrade request
        if let Some(key) = websocket::extract_websocket_key(data) {
            let mut response = [0u8; 256];
            let resp_len = websocket::build_upgrade_response(key, &mut response);
            if resp_len > 0 {
                let _ = socket.send_slice(&response[..resp_len]);
                WS_UPGRADED = true;
                crate::kprintln!("  NET: WebSocket upgraded on :7420");

                // Notify openDAQ about new client
                crate::daq::opendaq::on_client_connected();
            }
        }
        return;
    }

    // WebSocket is upgraded — parse binary frames
    if let Some((opcode, payload_offset, payload_len, mask_key)) =
        websocket::parse_frame_header(data)
    {
        if opcode == 0x02 && payload_offset + payload_len <= n {
            // Binary frame
            let mut payload = [0u8; 4096];
            let plen = payload_len.min(payload.len());
            payload[..plen].copy_from_slice(&data[payload_offset..payload_offset + plen]);

            // Unmask if client sent masked data
            if let Some(key) = mask_key {
                websocket::unmask_payload(key, &mut payload[..plen]);
            }

            // Forward to openDAQ protocol handler
            crate::daq::opendaq::on_client_message(0, &payload[..plen]);
        } else if opcode == 0x08 {
            // Close frame
            WS_UPGRADED = false;
            socket.close();
        } else if opcode == 0x09 {
            // Ping → respond with Pong (opcode 0x0A)
            let mut pong = [0u8; 128];
            pong[0] = 0x8A; // FIN + Pong
            pong[1] = 0;    // No payload
            let _ = socket.send_slice(&pong[..2]);
        }
    }
}

/// Send data over the WebSocket connection (for openDAQ streaming)
pub fn ws_send(data: &[u8]) -> Result<(), &'static str> {
    if !NET_READY.load(Ordering::Relaxed) { return Err("net not ready"); }

    unsafe {
        if !WS_UPGRADED { return Err("WebSocket not connected"); }

        let sockets = SOCKETS.as_mut().unwrap();
        let tcp_h = TCP_HANDLE.unwrap();
        let socket = sockets.get_mut::<tcp::Socket>(tcp_h);

        if !socket.can_send() { return Err("TCP send buffer full"); }

        // Wrap in WebSocket binary frame
        let mut frame = [0u8; 32768]; // Max frame size
        let frame_len = websocket::pack_binary_frame(data, &mut frame)
            .map_err(|_| "frame too large")?;

        socket.send_slice(&frame[..frame_len]).map_err(|_| "send error")?;
    }

    Ok(())
}
