use std::io::{Read, Result, Write};
use std::net::{TcpListener, TcpStream};
use std::os::unix::net::{UnixListener, UnixStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use log::{debug, error, info, warn};

use crate::framebuffer::FramebufferSurface;

/// VNC input event types
#[derive(Debug, Clone)]
pub enum VncInputEvent {
    Keyboard { key: u32, down: bool },
    MouseButton { button: u8, down: bool },
    PointerMove { dx: i16, dy: i16 },
    PointerPosition { x: u32, y: u32 },
}

/// Unified stream type for TCP and Unix sockets
pub enum VncStream {
    Tcp(TcpStream),
    Unix(UnixStream),
}

impl VncStream {
    fn set_nonblocking(&self, nonblocking: bool) -> Result<()> {
        match self {
            VncStream::Tcp(s) => s.set_nonblocking(nonblocking),
            VncStream::Unix(s) => s.set_nonblocking(nonblocking),
        }
    }

    fn set_read_timeout(&self, dur: Option<Duration>) -> Result<()> {
        match self {
            VncStream::Tcp(s) => s.set_read_timeout(dur),
            VncStream::Unix(s) => s.set_read_timeout(dur),
        }
    }

    fn as_raw_fd(&self) -> std::os::unix::io::RawFd {
        match self {
            VncStream::Tcp(s) => std::os::unix::io::AsRawFd::as_raw_fd(s),
            VncStream::Unix(s) => std::os::unix::io::AsRawFd::as_raw_fd(s),
        }
    }
}

impl Read for VncStream {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        match self {
            VncStream::Tcp(s) => s.read(buf),
            VncStream::Unix(s) => s.read(buf),
        }
    }
}

impl Write for VncStream {
    fn write(&mut self, buf: &[u8]) -> Result<usize> {
        match self {
            VncStream::Tcp(s) => s.write(buf),
            VncStream::Unix(s) => s.write(buf),
        }
    }

    fn flush(&mut self) -> Result<()> {
        match self {
            VncStream::Tcp(s) => s.flush(),
            VncStream::Unix(s) => s.flush(),
        }
    }
}

/// Unified listener type
pub enum VncListener {
    Tcp(TcpListener),
    Unix(UnixListener),
}

impl VncListener {
    fn accept(&self) -> Result<(VncStream, String)> {
        match self {
            VncListener::Tcp(l) => {
                let (stream, addr) = l.accept()?;
                Ok((VncStream::Tcp(stream), addr.to_string()))
            }
            VncListener::Unix(l) => {
                let (stream, addr) = l.accept()?;
                Ok((
                    VncStream::Unix(stream),
                    addr.as_pathname()
                        .map(|p| p.to_string_lossy().to_string())
                        .unwrap_or_else(|| "unix".to_string()),
                ))
            }
        }
    }

    fn set_nonblocking(&self, nonblocking: bool) -> Result<()> {
        match self {
            VncListener::Tcp(l) => l.set_nonblocking(nonblocking),
            VncListener::Unix(l) => l.set_nonblocking(nonblocking),
        }
    }
}

/// VNC listener configuration
#[derive(Debug, Clone)]
pub enum VncListenerType {
    Tcp { port: u16 },
    Unix { path: String },
}

/// VNC server configuration
#[derive(Clone)]
pub struct VncServerConfig {
    pub listener: VncListenerType,
}

/// VNC server that polls the framebuffer and serves a single client.
pub struct VncServer {
    surface: Arc<FramebufferSurface>,
    config: VncServerConfig,
    input_sender: mpsc::Sender<VncInputEvent>,
    running: Arc<AtomicBool>,
    on_disconnect: Mutex<Option<Box<dyn Fn() + Send>>>,
}

impl VncServer {
    pub fn new(
        surface: Arc<FramebufferSurface>,
        config: VncServerConfig,
        input_sender: mpsc::Sender<VncInputEvent>,
    ) -> Self {
        Self {
            surface,
            config,
            input_sender,
            running: Arc::new(AtomicBool::new(false)),
            on_disconnect: Mutex::new(None),
        }
    }

    pub fn set_on_disconnect<F>(&self, f: F)
    where
        F: Fn() + Send + 'static,
    {
        *self.on_disconnect.lock().unwrap() = Some(Box::new(f));
    }

    pub fn spawn(&self) -> std::io::Result<thread::JoinHandle<()>> {
        let surface = Arc::clone(&self.surface);
        let config = self.config.clone();
        let input_sender = self.input_sender.clone();
        let running = Arc::clone(&self.running);
        let on_disconnect = self.on_disconnect.lock().map_err(|e| {
            std::io::Error::other(format!("mutex poisoned: {e}"))
        })?.take();

        self.running.store(true, Ordering::SeqCst);

        thread::Builder::new()
            .name("ch-vnc-server".to_string())
            .spawn(move || {
                run_vnc_server(surface, config, input_sender, running, on_disconnect);
            })
            .map_err(|e| {
                error!("vnc: failed to spawn server thread: {e}");
                std::io::Error::other(format!("failed to spawn thread: {e}"))
            })
    }

    pub fn stop(&self) {
        self.running.store(false, Ordering::SeqCst);
    }
}

fn run_vnc_server(
    surface: Arc<FramebufferSurface>,
    config: VncServerConfig,
    input_sender: mpsc::Sender<VncInputEvent>,
    running: Arc<AtomicBool>,
    on_disconnect: Option<Box<dyn Fn() + Send>>,
) {
    let listener = match &config.listener {
        VncListenerType::Tcp { port } => match TcpListener::bind(format!("127.0.0.1:{port}")) {
            Ok(l) => {
                info!("vnc: listening on TCP 127.0.0.1:{port}");
                VncListener::Tcp(l)
            }
            Err(e) => {
                error!("vnc: failed to bind TCP port {port}: {e}");
                return;
            }
        },
        VncListenerType::Unix { path } => {
            let _ = std::fs::remove_file(path);
            match UnixListener::bind(path) {
                Ok(l) => {
                    info!("vnc: listening on Unix socket {path}");
                    VncListener::Unix(l)
                }
                Err(e) => {
                    error!("vnc: failed to bind Unix socket {path}: {e}");
                    return;
                }
            }
        }
    };

    listener.set_nonblocking(true).ok();

    while running.load(Ordering::SeqCst) {
        if surface.is_initialized() {
            debug!("vnc: surface initialized, breaking wait loop");
            break;
        }
        thread::sleep(Duration::from_millis(100));
    }

    if !running.load(Ordering::SeqCst) {
        return;
    }

    loop {
        if !running.load(Ordering::SeqCst) {
            break;
        }

        match listener.accept() {
            Ok((stream, peer)) => {
                info!("vnc: client connected from {peer}");
                match handle_client(
                    stream,
                    &surface,
                    &input_sender,
                    &running,
                    &on_disconnect,
                ) {
                    Ok(()) => info!("vnc: client disconnected from {peer}"),
                    Err(e) => warn!("vnc: handle_client error from {peer}: {e}"),
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(100));
            }
            Err(e) => {
                error!("vnc: accept failed: {e}");
                thread::sleep(Duration::from_millis(100));
            }
        }
    }
}

fn handle_client(
    stream: VncStream,
    surface: &Arc<FramebufferSurface>,
    input_sender: &mpsc::Sender<VncInputEvent>,
    running: &Arc<AtomicBool>,
    on_disconnect: &Option<Box<dyn Fn() + Send>>,
) -> Result<()> {
    let stream = Arc::new(Mutex::new(stream));

    {
        let mut s = stream.lock().unwrap();
        s.set_nonblocking(false)?;
        s.set_read_timeout(Some(Duration::from_secs(5))).ok();

        let use_rfb38 = handshake(&mut *s)?;
        security(&mut *s, use_rfb38)?;

        let config = surface.config();
        let width = config.width;
        let height = config.height;
        debug!("vnc: surface config width={} height={}", width, height);
        server_init(&mut *s, width, height)?;
        client_init(&mut *s)?;
    }

    {
        let s = stream.lock().unwrap();
        s.set_nonblocking(true)?;
        s.set_read_timeout(None).ok();
    }

    info!("vnc: client authenticated and connected");

    // Send initial framebuffer update immediately so the client knows the display is alive.
    // Many VNC clients (e.g., TigerVNC) throttle keyboard/mouse input until they receive
    // at least one FBU, assuming the display is frozen otherwise.
    let mut last_data: Option<Vec<u8>> = {
        let mut s = stream.lock().unwrap();
        if let Some(init_data) = surface.read_framebuffer() {
            if send_framebuffer_update(&mut *s, &init_data, surface.config().width, surface.config().height).is_ok() {
                info!("vnc: sent initial framebuffer update");
                Some(init_data)
            } else {
                warn!("vnc: failed to send initial framebuffer update");
                None
            }
        } else {
            warn!("vnc: read_framebuffer returned None during initial update");
            None
        }
    };

    let fb_interval = Duration::from_millis(40); // 25 FPS for framebuffer updates
    let input_interval = Duration::from_millis(100); // 10 Hz for input polling

    while running.load(Ordering::SeqCst) {
        let sleep_duration = {
            let mut s = stream.lock().unwrap();

            // Check for client input FIRST (before FBU to avoid blocking)
            let fd = s.as_raw_fd();
            let mut poll_fd = libc::pollfd {
                fd,
                events: libc::POLLIN,
                revents: 0,
            };
            let polled = {
                // SAFETY: fd is a valid file descriptor from a live VncStream.
                // poll_fd is properly initialized with valid fields.
                // The pointer is valid for reads of 1 element.
                (unsafe { libc::poll(&mut poll_fd, 1, 0) }) > 0
            };
            if polled && (poll_fd.revents & (libc::POLLHUP | libc::POLLERR | libc::POLLNVAL)) != 0 {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::ConnectionReset,
                    "Client disconnected (poll hangup/error)",
                ));
            }
            let readable = polled && (poll_fd.revents & libc::POLLIN) != 0;

            if readable {
                // Keep non-blocking — check_client_input stops on WouldBlock/EAGAIN.
                info!("vnc: socket readable, draining input");
                if let Err(e) = check_client_input(&mut *s, input_sender) {
                    warn!("vnc: input read error: {}", e);
                }
            }

            let mut fb_sent = false;
            match surface.read_framebuffer() {
                Some(current_data) => {
                    let has_change = match &last_data {
                        None => true,
                        Some(prev) => prev != &current_data,
                    };
                    debug!(
                        "vnc: read_framebuffer returned {} bytes, has_change={}",
                        current_data.len(),
                        has_change
                    );

                    if has_change {
                        // Check if socket is writable before sending FBU
                        let mut poll_out = libc::pollfd {
                            fd: s.as_raw_fd(),
                            events: libc::POLLOUT,
                            revents: 0,
                        };
                        let writable = {
                            // SAFETY: fd is valid from live VncStream
                            (unsafe { libc::poll(&mut poll_out, 1, 0) }) > 0
                        } && (poll_out.revents & libc::POLLOUT) != 0;

                        if writable {
                            if send_framebuffer_update(
                                &mut *s,
                                &current_data,
                                surface.config().width,
                                surface.config().height,
                            )
                            .is_err()
                            {
                                warn!("vnc: failed to send framebuffer update, client disconnected");
                                return Err(std::io::Error::new(
                                    std::io::ErrorKind::BrokenPipe,
                                    "Client disconnected",
                                ));
                            } else {
                                last_data = Some(current_data);
                                fb_sent = true;
                            }
                        }
                    }
                }
                None => {
                    debug!("vnc: read_framebuffer returned None");
                }
            }
            if fb_sent {
                fb_interval
            } else {
                input_interval
            }
        }; // drop lock

        thread::sleep(sleep_duration);
    }

    if let Some(cb) = on_disconnect {
        cb();
    }

    Ok(())
}

fn handshake<RW: Read + Write>(rw: &mut RW) -> Result<bool> {
    // Advertise RFB 3.8 for list-based security negotiation.
    rw.write_all(b"RFB 003.008\n")?;
    rw.flush()?;

    let mut client_version = [0u8; 12];
    rw.read_exact(&mut client_version)?;

    let client_ver_str = String::from_utf8_lossy(&client_version);
    info!(
        "vnc: client version: {} (bytes: {:02x?}",
        client_ver_str.trim(),
        client_version
    );

    // Parse client version string: "RFB 003.XXX\n" or multi-version "RFB 003.089\n003.079\n..."
    // Check if client supports RFB 3.8 (003.008)
    let uses_rfb38 = client_ver_str.contains("003.008");

    if uses_rfb38 {
        info!("vnc: using RFB 3.8 protocol");
    } else if client_ver_str.contains("003.003") {
        info!("vnc: using RFB 3.3 protocol");
    } else {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!("Unsupported RFB version: {}", client_ver_str.trim()),
        ));
    }

    Ok(uses_rfb38)
}

fn security<RW: Read + Write>(rw: &mut RW, use_rfb38: bool) -> Result<()> {
    if use_rfb38 {
        // RFB 3.8 security negotiation:
        // Server sends: 1-byte count + count x 1-byte security types
        // Client responds with: 1-byte selected security type
        // Server sends: 4-byte security result (0=OK)
        rw.write_all(&[1])?; // count = 1 (1 byte)
        rw.write_all(&[1])?; // type 1 = None (no authentication)
        rw.flush()?;
        info!("vnc: sent RFB 3.8 security types (count=1, type=1=None), waiting for client response");
    } else {
        // RFB 3.3 security negotiation:
        // Server sends: 4-byte security type directly (no count byte)
        // - type 1 = VNC Authentication (requires password challenge, which we don't support)
        // - For backward compatibility with clients that only support 3.3, we still need to
        //   handle this, but it will fail without proper VNC auth implementation.
        //   Since we advertise 3.8 first, most clients will negotiate to 3.8.
        rw.write_all(&[0, 0, 0, 1])?; // type 1 = VNC Authentication (RFB 3.3 format, no count)
        rw.flush()?;
        info!("vnc: sent RFB 3.3 security type (1=VNCAuth), waiting for client response");
    }

    let mut selected = [0u8; 1];
    match rw.read_exact(&mut selected) {
        Ok(()) => {},
        Err(e) => {
            info!("vnc: read_exact for security type failed: {e}");
            return Err(e);
        }
    }

    let selected_type = u32::from(selected[0]);
    debug!("vnc: client selected security type: {selected_type}");

    if use_rfb38 {
        if selected_type != 1 {
            rw.write_all(&[0, 0, 0, 1])?; // 1 security reason
            rw.write_all(b"Invalid security type")?;
            rw.flush()?;
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "Invalid security type",
            ));
        }
        // Auth OK - send 4-byte security result
        rw.write_all(&[0, 0, 0, 0])?; // result 0 = OK
        rw.flush()?;
    } else {
        if selected_type != 1 {
            rw.write_all(&[0, 0, 0, 1])?;
            rw.write_all(b"Invalid security type")?;
            rw.flush()?;
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "Invalid security type",
            ));
        }
        // For RFB 3.3, type 1 = VNC Auth, which requires sending a 16-byte challenge.
        // We don't support VNC auth, so this path will fail. Clients should negotiate to 3.8.
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "VNC authentication not supported; client must support RFB 3.8",
        ));
    }

    Ok(())
}

fn server_init<W: Write>(writer: &mut W, width: u32, height: u32) -> Result<()> {
    let name = b"Cloud Hypervisor";
    let name_len = name.len() as u32;

    // RFB ServerInit: width and height are CARD16 (2 bytes each)
    writer.write_all(&((width as u16).to_be_bytes()))?;
    writer.write_all(&((height as u16).to_be_bytes()))?;

    // Pixel format: XRGB8888 (16 bytes)
    // bits_per_pixel=32, depth=24, big_endian=0, true_color=1
    // red_max=255, green_max=255, blue_max=255
    // red_shift=16, green_shift=8, blue_shift=0
    // Note: TigerVNC reads shift values as 1 byte each + 3 padding bytes
    writer.write_all(&[32, 24, 0, 1, 0, 255, 0, 255, 0, 255, 16, 8, 0, 0, 0, 0])?;

    writer.write_all(&name_len.to_be_bytes())?;
    writer.write_all(name)?;
    writer.flush()?;

    Ok(())
}

fn client_init<R: Read>(reader: &mut R) -> Result<()> {
    let mut shared = [0u8; 1];
    reader.read_exact(&mut shared)?;

    let shared_flag = shared[0];
    debug!("vnc: client shared flag: {shared_flag}");

    Ok(())
}

fn send_framebuffer_update<W: Write>(
    writer: &mut W,
    data: &[u8],
    width: u32,
    height: u32,
) -> Result<()> {
    let bytes_per_pixel = 4u32;
    let stride = width * bytes_per_pixel;
    let fb_size = stride * height;

    if data.len() < fb_size as usize {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "Framebuffer data too small",
        ));
    }

    // Framebuffer update message:
    // message-type=0 (1 byte), padding (1 byte), number-of-rects (CARD16)
    writer.write_all(&[0])?;
    writer.write_all(&[0])?;
    writer.write_all(&[0, 1])?;

    // Rectangle: x, y, width, height (all CARD16), encoding (CARD32)
    writer.write_all(&[0, 0])?;         // x = 0
    writer.write_all(&[0, 0])?;         // y = 0
    writer.write_all(&((width as u16).to_be_bytes()))?;    // width CARD16
    writer.write_all(&((height as u16).to_be_bytes()))?;   // height CARD16
    writer.write_all(&[0, 0, 0, 0])?;  // encoding = 0 (RAW)

    writer.write_all(&data[..fb_size as usize])?;
    writer.flush()?;

    Ok(())
}

fn check_client_input<R: Read>(
    reader: &mut R,
    input_sender: &mpsc::Sender<VncInputEvent>,
) -> Result<()> {
    loop {
        let mut msg_type = [0u8; 1];
        if reader.read_exact(&mut msg_type).is_err() {
            break;
        }

        match msg_type[0] {
            0 => {
                let mut rest = [0u8; 23];
                if reader.read_exact(&mut rest).is_err() {
                    break;
                }
                debug!("vnc: client sent SetPixelFormat");
            }
            1 => {
                let mut header = [0u8; 6];
                if reader.read_exact(&mut header).is_err() {
                    break;
                }
                let n_entries = u16::from_be_bytes([header[4], header[5]]) as usize;
                let entry_size = 6 * n_entries;
                let mut entries = vec![0u8; entry_size];
                if reader.read_exact(&mut entries).is_err() {
                    break;
                }
                let first_idx = u16::from_be_bytes([header[2], header[3]]);
                debug!("vnc: SetColorMapEntries first={first_idx} n={n_entries}");
            }
            2 => {
                let mut header = [0u8; 4];
                if reader.read_exact(&mut header).is_err() {
                    break;
                }
                let count = u16::from_be_bytes([header[2], header[3]]) as usize;
                let mut encodings = vec![0u8; 4 * count];
                if reader.read_exact(&mut encodings).is_err() {
                    break;
                }
                debug!("vnc: SetEncodings count={count}");
            }
            3 => {
                let mut header = [0u8; 4];
                if reader.read_exact(&mut header).is_err() {
                    break;
                }
                let inc = header[1] != 0;
                let num_rects = u16::from_be_bytes([header[2], header[3]]) as usize;
                debug!("vnc: FramebufferUpdateRequest incremental={inc} num_rects={num_rects}");
                if num_rects > 0 {
                    let mut rect_data = vec![0u8; num_rects * 12];
                    let _ = reader.read_exact(&mut rect_data);
                }
            }
            4 => {
                let mut padding = [0u8; 3];
                let mut key_data = [0u8; 4];
                if reader.read_exact(&mut padding).is_err()
                    || reader.read_exact(&mut key_data).is_err()
                {
                    break;
                }
                let down = padding[0] != 0;
                let key = u32::from_be_bytes(key_data);

                let _ = input_sender.send(VncInputEvent::Keyboard { key, down });
                info!("vnc: key event keysym=0x{key:x} down={down}");
            }
            5 => {
                let mut pointer_data = [0u8; 4];
                if reader.read_exact(&mut pointer_data).is_err() {
                    break;
                }
                let mask = pointer_data[0];
                let mut pos = [0u8; 4];
                if reader.read_exact(&mut pos).is_err() {
                    break;
                }
                let x = u16::from_be_bytes([pos[0], pos[1]]) as u32;
                let y = u16::from_be_bytes([pos[2], pos[3]]) as u32;

                let left_down = (mask & 1) != 0;
                let middle_down = (mask & 2) != 0;
                let right_down = (mask & 4) != 0;

                let _ = input_sender.send(VncInputEvent::MouseButton {
                    button: 0,
                    down: left_down,
                });
                let _ = input_sender.send(VncInputEvent::MouseButton {
                    button: 1,
                    down: middle_down,
                });
                let _ = input_sender.send(VncInputEvent::MouseButton {
                    button: 2,
                    down: right_down,
                });
                let _ = input_sender.send(VncInputEvent::PointerPosition { x, y });

                debug!("vnc: pointer event mask={mask} x={x} y={y}");
            }
            6 => {
                let mut header = [0u8; 4];
                if reader.read_exact(&mut header).is_err() {
                    break;
                }
                let length = u32::from_be_bytes(header) as usize;
                if length > 0 {
                    let mut text = vec![0u8; length];
                    if reader.read_exact(&mut text).is_err() {
                        break;
                    }
                    debug!("vnc: ClientCutText length={length}");
                }
            }
            9 => {
                let mut header = [0u8; 4];
                if reader.read_exact(&mut header).is_err() {
                    break;
                }
                let first_color = u16::from_be_bytes([header[0], header[1]]);
                let num_colors = u16::from_be_bytes([header[2], header[3]]) as usize;
                let mut entries = vec![0u8; 6 * num_colors];
                if reader.read_exact(&mut entries).is_err() {
                    break;
                }
                debug!("vnc: SetColourValues first={first_color} num={num_colors}");
            }
            // QEMU Extended Client Message (TigerVNC preferred format)
            0xFF => {
                let mut data = [0u8; 12];
                if reader.read_exact(&mut data).is_err() {
                    break;
                }
                let sub_type = data[0];
                if sub_type == 0 {
                    // QEMU Extended KeyEvent:
                    // sub-type(1) + down-flag(U16) + keysym(U32) + keycode(U32) = 12 bytes
                    let down = u16::from_be_bytes([data[1], data[2]]) != 0;
                    let key = u32::from_be_bytes([data[3], data[4], data[5], data[6]]);
                    let _ = input_sender.send(VncInputEvent::Keyboard { key, down });
                    info!("vnc: qemu key event keysym=0x{key:x} down={down}");
                } else {
                    info!("vnc: qemu extended msg sub-type={sub_type}");
                }
            }
            _ => {
                info!("vnc: unknown message type 0x{:02x}", msg_type[0]);
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    struct MockSocket {
        read_buf: Cursor<Vec<u8>>,
        write_buf: Vec<u8>,
    }

    impl MockSocket {
        fn new() -> Self {
            Self {
                read_buf: Cursor::new(Vec::new()),
                write_buf: Vec::new(),
            }
        }

        fn queue_read_data(&mut self, data: &[u8]) {
            self.read_buf = Cursor::new(data.to_vec());
        }

        fn written_data(&self) -> &[u8] {
            &self.write_buf
        }
    }

    impl Read for MockSocket {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            self.read_buf.read(buf)
        }
    }

    impl Write for MockSocket {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.write_buf.extend_from_slice(buf);
            Ok(buf.len())
        }

        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn test_rfb38_handshake() {
        let mut sock = MockSocket::new();
        sock.queue_read_data(b"RFB 003.008\n");
        let result = handshake(&mut sock);
        assert!(result.is_ok(), "handshake should succeed for RFB 3.8");
        assert!(result.unwrap(), "should negotiate RFB 3.8");
        assert!(sock.written_data().starts_with(b"RFB 003.008\n"));
    }

    #[test]
    fn test_rfb33_handshake() {
        let mut sock = MockSocket::new();
        sock.queue_read_data(b"RFB 003.003\n");
        let result = handshake(&mut sock);
        assert!(result.is_ok());
        assert!(!result.unwrap(), "should fall back to RFB 3.3");
    }

    #[test]
    fn test_rfb38_security_noauth() {
        let mut sock = MockSocket::new();
        sock.queue_read_data(&[1]);
        security(&mut sock, true).expect("security should succeed for type 1 (None)");
        let w = sock.written_data();
        assert_eq!(&w[0..1], &[1]); // count=1
        assert_eq!(&w[1..2], &[1]); // type=None (1 byte in RFB 3.8)
        assert_eq!(&w[2..6], &[0, 0, 0, 0]); // OK (4-byte result)
    }

    #[test]
    fn test_full_rfb38_handshake() {
        let mut sock = MockSocket::new();

        sock.queue_read_data(b"RFB 003.008\n");
        let use_rfb38 = handshake(&mut sock).expect("handshake failed");
        assert!(use_rfb38);

        sock.queue_read_data(&[1]);
        security(&mut sock, use_rfb38).expect("security failed");

        sock.queue_read_data(&[]);
        server_init(&mut sock, 800, 600).expect("server_init failed");

        sock.queue_read_data(&[1]);
        client_init(&mut sock).expect("client_init failed");

        let w = sock.written_data();
        assert!(w.starts_with(b"RFB 003.008\n"));
        // Security types start at offset 12 (after "RFB 003.008\n")
        assert_eq!(&w[12..13], &[1]); // count=1
        assert_eq!(&w[13..14], &[1]); // type=None
        assert_eq!(&w[14..18], &[0, 0, 0, 0]); // OK
        // Server init starts at offset 18
        assert_eq!(&w[18..20], &(800u16).to_be_bytes());
        assert_eq!(&w[20..22], &(600u16).to_be_bytes());
    }
}
