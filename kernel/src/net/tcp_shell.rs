//! TCP Remote Shell for Folkering DAQ — port 2222.
//!
//! Plaintext shell for remote inspection and control. Commands:
//! help, ps, uptime, mem, net, daq status, daq start/stop, ping

use smoltcp::socket::tcp;
use smoltcp::iface::SocketHandle;

const SHELL_PORT: u16 = 2222;
const BANNER: &[u8] = b"\r\n  Folkering DAQ v1.0 - TCP Shell\r\n  Type 'help' for commands.\r\n\r\n> ";

// Static buffers for the shell socket (no heap allocation)
static mut SHELL_RX_BUF: [u8; 4096] = [0; 4096];
static mut SHELL_TX_BUF: [u8; 4096] = [0; 4096];

static mut SHELL_HANDLE: Option<SocketHandle> = None;
static mut SHELL_LINE_BUF: [u8; 256] = [0; 256];
static mut SHELL_LINE_LEN: usize = 0;
static mut SHELL_CONNECTED: bool = false;

/// Initialize the TCP shell. Call after net::init().
pub unsafe fn init(sockets: &mut smoltcp::iface::SocketSet<'static>) {
    let tcp_rx = tcp::SocketBuffer::new(&mut SHELL_RX_BUF[..]);
    let tcp_tx = tcp::SocketBuffer::new(&mut SHELL_TX_BUF[..]);
    let mut socket = tcp::Socket::new(tcp_rx, tcp_tx);

    if let Err(_) = socket.listen(SHELL_PORT) {
        crate::kprintln!("  SHELL: failed to listen on port 2222");
        return;
    }

    let handle = sockets.add(socket);
    SHELL_HANDLE = Some(handle);
    SHELL_CONNECTED = false;
    SHELL_LINE_LEN = 0;

    crate::kprintln!("  SHELL: listening on port 2222");
}

/// Poll the shell. Called from net::poll() with all state available.
pub unsafe fn poll(sockets: &mut smoltcp::iface::SocketSet) {
    let handle = match SHELL_HANDLE {
        Some(h) => h,
        None => return,
    };

    let socket = sockets.get_mut::<tcp::Socket>(handle);

    // Recovery: stale socket → re-listen
    if !SHELL_CONNECTED && !socket.is_active() && !socket.is_listening() {
        socket.abort();
        let _ = socket.listen(SHELL_PORT);
        return;
    }

    // New connection
    if !SHELL_CONNECTED && socket.is_active() && socket.may_send() {
        SHELL_CONNECTED = true;
        SHELL_LINE_LEN = 0;
        let _ = socket.send_slice(BANNER);
        crate::kprintln!("  SHELL: client connected");
        return;
    }

    // Client disconnected
    if SHELL_CONNECTED && (!socket.is_active() || !socket.may_recv()) {
        SHELL_CONNECTED = false;
        SHELL_LINE_LEN = 0;
        crate::kprintln!("  SHELL: client disconnected");
        socket.abort();
        let _ = socket.listen(SHELL_PORT);
        return;
    }

    // Read incoming data
    if SHELL_CONNECTED && socket.can_recv() {
        let mut tmp = [0u8; 256];
        let n = socket.recv_slice(&mut tmp).unwrap_or(0);

        for i in 0..n {
            let byte = tmp[i];
            if byte >= 0xF0 { continue; } // Telnet IAC
            if byte == b'\r' { continue; }

            if byte == b'\n' {
                let socket = sockets.get_mut::<tcp::Socket>(handle);
                let _ = socket.send_slice(b"\r\n");

                if SHELL_LINE_LEN > 0 {
                    let response = dispatch(&SHELL_LINE_BUF[..SHELL_LINE_LEN]);
                    let socket = sockets.get_mut::<tcp::Socket>(handle);
                    let _ = socket.send_slice(response.as_bytes());
                    let _ = socket.send_slice(b"\r\n");
                }
                let socket = sockets.get_mut::<tcp::Socket>(handle);
                let _ = socket.send_slice(b"> ");
                SHELL_LINE_LEN = 0;
            } else if byte == 0x7F || byte == 0x08 {
                // Backspace
                if SHELL_LINE_LEN > 0 {
                    SHELL_LINE_LEN -= 1;
                    let socket = sockets.get_mut::<tcp::Socket>(handle);
                    let _ = socket.send_slice(b"\x08 \x08");
                }
            } else if SHELL_LINE_LEN < 255 {
                // Echo + store
                SHELL_LINE_BUF[SHELL_LINE_LEN] = byte;
                SHELL_LINE_LEN += 1;
                let socket = sockets.get_mut::<tcp::Socket>(handle);
                let _ = socket.send_slice(&[byte]);
            }
        }
    }
}

// ── Command Dispatch ────────────────────────────────────────────────

fn dispatch(line: &[u8]) -> &'static str {
    // Simple dispatch without heap allocation — returns static strings
    // For dynamic output, we'd need alloc. DAQ kernel might not have it.
    let cmd = match core::str::from_utf8(line) {
        Ok(s) => s.trim(),
        Err(_) => return "error: invalid UTF-8",
    };

    let (verb, args) = match cmd.split_once(' ') {
        Some((v, a)) => (v, a.trim()),
        None => (cmd, ""),
    };

    match verb {
        "help" => HELP_TEXT,
        "uptime" => "uptime: use 'uptime' (dynamic output requires alloc)",
        "daq" => dispatch_daq(args),
        "net" => dispatch_net(),
        "ping" => "ping: not yet implemented in DAQ build",
        "" => "",
        _ => "unknown command. Type 'help'",
    }
}

fn dispatch_daq(args: &str) -> &'static str {
    match args {
        "status" => {
            if crate::daq::opendaq::is_connected() {
                "DAQ: client connected, streaming active"
            } else {
                "DAQ: no client connected, idle"
            }
        }
        "stop" => {
            // TODO: implement graceful DAQ stop
            "DAQ: stop not implemented yet"
        }
        "start" => {
            "DAQ: start not implemented yet"
        }
        _ => "usage: daq status|start|stop",
    }
}

fn dispatch_net() -> &'static str {
    // IP is static in DAQ build
    "net: 192.168.1.100/24 (static)\r\n  TCP :7420 openDAQ\r\n  UDP :5353 mDNS\r\n  TCP :2222 shell"
}

const HELP_TEXT: &str = "\
Commands:\r\n\
  help         show this message\r\n\
  uptime       system uptime\r\n\
  net          network configuration\r\n\
  daq status   DAQ streaming status\r\n\
  daq start    start data acquisition\r\n\
  daq stop     stop data acquisition\r\n\
  ping <ip>    (not yet implemented)";
