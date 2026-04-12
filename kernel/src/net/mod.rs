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
pub mod tcp_shell;

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
static mut SOCKET_SET_BUF: [smoltcp::iface::SocketStorage<'static>; 5] =
    [smoltcp::iface::SocketStorage::EMPTY; 5];
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

    // Initialize TCP remote shell on port 2222
    unsafe { tcp_shell::init(SOCKETS.as_mut().unwrap()); }

    NET_READY.store(true, Ordering::SeqCst);
    crate::kprintln!("  NET: stack ready, TCP :7420 + UDP :5353 + TCP :2222 shell");
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

        // TCP remote shell
        tcp_shell::poll(sockets);
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

/// Handle openDAQ TCP connections on port 7420.
///
/// Protocol: daq.nd:// — raw TCP binary streaming (NOT WebSocket).
/// DewesoftX connects directly to raw TCP, sends minimal handshake,
/// then server pushes SignalAvailable JSON followed by binary DataPackets.
unsafe fn handle_opendaq_tcp(sockets: &mut SocketSet) {
    let tcp_h = match TCP_HANDLE { Some(h) => h, None => return };
    let socket = sockets.get_mut::<tcp::Socket>(tcp_h);

    if !socket.is_active() {
        if crate::daq::opendaq::is_connected() {
            crate::daq::opendaq::on_client_disconnected();
        }
        let _ = socket.listen(7420);
        return;
    }

    // Disable Nagle's algorithm — CRITICAL for real-time streaming
    socket.set_nagle_enabled(false);

    // New connection: send metadata immediately
    if socket.is_active() && !crate::daq::opendaq::is_connected() {
        crate::daq::opendaq::on_client_connected();

        // Send all SignalAvailable JSON payloads over raw TCP
        let payloads = crate::daq::opendaq::get_metadata_payloads();
        for payload in &payloads {
            if socket.can_send() {
                // Prefix each JSON with 4-byte LE length (simple framing)
                let len_bytes = (payload.len() as u32).to_le_bytes();
                let _ = socket.send_slice(&len_bytes);
                let _ = socket.send_slice(payload);
            }
        }
    }

    // Read client data (subscription commands, keepalive)
    if socket.may_recv() {
        let mut recv_buf = [0u8; 2048];
        if let Ok(n) = socket.recv_slice(&mut recv_buf) {
            if n > 0 {
                crate::daq::opendaq::on_client_message(&recv_buf[..n]);
            }
        }
    }
}

/// Send raw binary data directly over TCP (no WebSocket framing).
/// Used by openDAQ DataPacket transmission (0xDA51 sync word packets).
///
/// TCP_NODELAY ensures each packet is flushed immediately.
/// If send buffer is full, returns Err — caller should drop oldest data.
pub fn tcp_send_raw(data: &[u8]) -> Result<(), &'static str> {
    if !NET_READY.load(Ordering::Relaxed) { return Err("net not ready"); }

    unsafe {
        let sockets = SOCKETS.as_mut().unwrap();
        let tcp_h = TCP_HANDLE.unwrap();
        let socket = sockets.get_mut::<tcp::Socket>(tcp_h);

        if !socket.can_send() { return Err("send buffer full"); }

        socket.send_slice(data).map_err(|_| "send error")?;
    }

    Ok(())
}

/// Legacy WebSocket send (kept for future browser-based clients)
pub fn ws_send(data: &[u8]) -> Result<(), &'static str> {
    // For daq.nd:// protocol, just send raw TCP
    tcp_send_raw(data)
}
