// Copyright 2017 The Chromium OS Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE-BSD-3-Clause file.
//
// SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Barrier, Mutex};
use std::thread;

use log::{debug, error, info, warn};
use vm_device::BusDevice;
use vmm_sys_util::eventfd::EventFd;

// I/O port offsets relative to base 0x60
const OFFSET_DATA: u64 = 0;
const OFFSET_PORT_B: u64 = 1;
const OFFSET_TIMER_LATCH: u64 = 2;
const OFFSET_MODE_SELECT: u64 = 3;
const OFFSET_COMMAND: u64 = 4;

// PS/2 command codes
const CMD_READ_PORT_A: u8 = 0x20;
const CMD_READ_PORT_B: u8 = 0x21;
const CMD_TEST_CONTROLLER: u8 = 0xAA;
const CMD_DISABLE_SECONDARY: u8 = 0xA7;
const CMD_DISABLE_KEYBOARD: u8 = 0xAD;
const CMD_ENABLE_KEYBOARD: u8 = 0xAE;
const CMD_ENABLE_SECONDARY: u8 = 0xA8;
const CMD_READ_KBD_INPUT: u8 = 0xD0;
const CMD_TEST_KEYBOARD: u8 = 0xD1;
const CMD_WRITE_KBD_OUTPUT: u8 = 0xD2;
const CMD_WRITE_SECONDARY_OUTPUT: u8 = 0xD3;
const CMD_SELF_TEST: u8 = 0xF0;
const CMD_RESET_CONTROLLER: u8 = 0xFE;

// PS/2 status register bits
const STATUS_OUTPUT_BUFFER_FULL: u8 = 1 << 0;
#[allow(dead_code)]
const STATUS_INPUT_BUFFER_FULL: u8 = 1 << 1;
const STATUS_SYSTEM_FLAG: u8 = 1 << 2;
#[allow(dead_code)]
const STATUS_COMMAND_DATA: u8 = 1 << 3;
#[allow(dead_code)]
const STATUS_TIMEOUT: u8 = 1 << 5;
#[allow(dead_code)]
const STATUS_PARITY_ERROR: u8 = 1 << 6;
#[allow(dead_code)]
const STATUS_RTC: u8 = 1 << 7;

// PS/2 keyboard IRQ line
#[allow(dead_code)]
const KBD_IRQ: u8 = 1;
// PS/2 mouse IRQ line
#[allow(dead_code)]
const AUX_IRQ: u8 = 12;

// Maximum data buffer size
const MAX_DATA_BUFFER: usize = 32;

/// Input event types that can be forwarded to the guest via PS/2.
#[derive(Debug, Clone, Copy)]
pub enum InputEvent {
    /// Keyboard event with a virtual key code and pressed state.
    Keyboard { key: u32, pressed: bool },
    /// Mouse event with button state and X/Y movement.
    Mouse { buttons: u8, dx: i16, dy: i16 },
}

/// PS/2 keyboard scan code set 1 (IBM PC/AT compatible).
/// Maps virtual key codes to make/break scan codes.
struct KeyboardMap;

impl KeyboardMap {
    /// Generate scan code(s) for a key press or release.
    /// Returns a Vec of bytes to send to the PS/2 data port.
    fn scan_codes(key: u32, pressed: bool) -> Vec<u8> {
        let make_code = match key {
            // Esc
            0x01 => 0x01,
            // 1-0
            0x02 => 0x02,
            0x03 => 0x03,
            0x04 => 0x04,
            0x05 => 0x05,
            0x06 => 0x06,
            0x07 => 0x07,
            0x08 => 0x08,
            0x09 => 0x09,
            0x0A => 0x0A,
            // Q-P
            0x10 => 0x10,
            0x11 => 0x11,
            0x12 => 0x12,
            0x13 => 0x13,
            0x14 => 0x14,
            0x15 => 0x15,
            0x16 => 0x16,
            0x17 => 0x17,
            0x18 => 0x18,
            0x19 => 0x19,
            // A-L
            0x1E => 0x1E,
            0x1F => 0x1F,
            0x20 => 0x20,
            0x21 => 0x21,
            0x22 => 0x22,
            0x23 => 0x23,
            0x24 => 0x24,
            0x25 => 0x25,
            0x26 => 0x26,
            0x27 => 0x27,
            // Z-M
            0x2C => 0x2C,
            0x2D => 0x2D,
            0x2E => 0x2E,
            0x2F => 0x2F,
            0x30 => 0x30,
            0x31 => 0x31,
            0x32 => 0x32,
            0x33 => 0x33,
            0x34 => 0x34,
            0x35 => 0x35,
            // Tab
            0x0F => 0x0F,
            // Enter
            0x1C => 0x1C,
            // L Shift
            0x2A => 0x2A,
            // L Ctrl
            0x1D => 0x1D,
            // L Alt
            0x38 => 0x38,
            // Space
            0x39 => 0x39,
            // R Shift
            0x36 => 0x36,
            // R Alt (Alt Gr)
            0xB8 => 0xB8,
            // Backspace
            0x0E => 0x0E,
            // Caps Lock
            0x3A => 0x3A,
            // Insert
            0x52 => 0x52,
            // Home
            0x47 => 0x47,
            // Page Up
            0x49 => 0x49,
            // Delete
            0x53 => 0x53,
            // End
            0x4F => 0x4F,
            // Page Down
            0x51 => 0x51,
            // Up Arrow
            0x48 => 0x48,
            // Down Arrow
            0x50 => 0x50,
            // Left Arrow
            0x4B => 0x4B,
            // Right Arrow
            0x4D => 0x4D,
            // F1-F12
            0x3B => 0x3B,
            0x3C => 0x3C,
            0x3D => 0x3D,
            0x3E => 0x3E,
            0x3F => 0x3F,
            0x40 => 0x40,
            0x41 => 0x41,
            0x42 => 0x42,
            0x43 => 0x43,
            0x44 => 0x44,
            0x45 => 0x45,
            0x46 => 0x46,
            // Print Screen
            0x37 => 0x37,
            // Numpad keys
            0x5B => 0x77,
            0x60 => 0x70,
            0x61 => 0x69,
            0x62 => 0x72,
            0x63 => 0x7A,
            0x64 => 0x6B,
            0x65 => 0x73,
            0x66 => 0x74,
            0x67 => 0x7C,
            0x68 => 0x71,
            0x69 => 0x79,
            0x6A => 0x7B,
            0x6B => 0x75,
            0x6C => 0x7D,
            0x6D => 0x76,
            0x6E => 0x73,
            0x6F => 0x77,
            // Grave
            0x29 => 0x29,
            // Left Bracket
            0x1A => 0x1A,
            // Backslash
            0x2B => 0x2B,
            // Right Bracket
            0x1B => 0x1B,
            // Quote
            0x28 => 0x28,
            // All other keys: fall back to a generic scan code
            _ => 0x00,
        };

        if make_code == 0 {
            return Vec::new();
        }

        if pressed {
            vec![make_code]
        } else {
            vec![0xF0, make_code]
        }
    }
}

/// A i8042 PS/2 controller that emulates keyboard and mouse input.
///
/// This device handles PS/2 protocol commands, keyboard scan code generation,
/// and mouse packet generation. Input events are received via `input_channel`
/// and forwarded to the guest through the PS/2 data port.
pub struct I8042Device {
    reset_evt: EventFd,
    vcpus_kill_signalled: Arc<AtomicBool>,
    vcpus_pause_signalled: Arc<AtomicBool>,

    // PS/2 status register
    status: u8,

    // Output data buffer (data to send to guest on read of 0x60)
    output_buffer: VecDeque<u8>,

    // Input buffer for command data (written to 0x60 during command sequence)
    input_buffer: Option<u8>,

    // Port A register (keyboard/mouse presence and status)
    // Bit 0: keyboard output buffer full / keyboard present
    // Bit 4: mouse output buffer full / mouse present
    port_a: u8,

    // Port B register value
    port_b: u8,

    // Keyboard state
    keyboard_enabled: bool,
    // Mouse (secondary) state
    mouse_enabled: bool,

    // Pending command that needs data from input buffer
    pending_command: Option<u8>,

    // Channel to receive input events from VNC or other sources
    input_channel: Option<Arc<Mutex<VecDeque<InputEvent>>>>,
}

impl I8042Device {
    /// Constructs a i8042 device.
    ///
    /// `reset_evt` - EventFd to signal on guest reset request.
    /// `vcpus_kill_signalled` - Flag indicating vCPUs have been signaled to stop.
    /// `vcpus_pause_signalled` - Flag indicating vCPUs have been signaled to pause.
    /// `input_channel` - Optional shared queue for receiving input events from VNC.
    pub fn new(
        reset_evt: EventFd,
        vcpus_kill_signalled: Arc<AtomicBool>,
        vcpus_pause_signalled: Arc<AtomicBool>,
        input_channel: Option<Arc<Mutex<VecDeque<InputEvent>>>>,
    ) -> I8042Device {
        I8042Device {
            reset_evt,
            vcpus_kill_signalled,
            vcpus_pause_signalled,
            status: STATUS_SYSTEM_FLAG,
            output_buffer: VecDeque::new(),
            input_buffer: None,
            port_a: 0x00,
            port_b: 0x23,
            keyboard_enabled: true,
            mouse_enabled: true,
            pending_command: None,
            input_channel,
        }
    }

    /// Push a byte to the output buffer and trigger interrupt if data was added.
    fn push_output(&mut self, byte: u8) {
        if self.output_buffer.len() < MAX_DATA_BUFFER {
            self.output_buffer.push_back(byte);
            self.status |= STATUS_OUTPUT_BUFFER_FULL;
        } else {
            debug!("i8042: output buffer full, dropping byte 0x{byte:02x}");
        }
    }

    /// Push multiple bytes to the output buffer.
    fn push_output_bytes(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.push_output(b);
        }
    }

    /// Process a keyboard input event and generate scan codes.
    fn process_keyboard_event(&mut self, key: u32, pressed: bool) {
        if !self.keyboard_enabled {
            return;
        }
        let scan_codes = KeyboardMap::scan_codes(key, pressed);
        if !scan_codes.is_empty() {
            self.push_output_bytes(&scan_codes);
        }
    }

    /// Process a mouse input event and generate a 3-byte mouse packet.
    ///
    /// Standard PS/2 mouse packet format:
    /// Byte 0: bits 0-2 = buttons (L, R, M), bit 3 = Y overflow, bit 4 = X overflow,
    ///         bit 5 = Y sign, bit 6 = X sign, bit 7 = 1 (always)
    /// Byte 1: X movement (signed, 8-bit with overflow flag)
    /// Byte 2: Y movement (signed, 8-bit with overflow flag)
    fn process_mouse_event(&mut self, buttons: u8, dx: i16, dy: i16) {
        if !self.mouse_enabled {
            return;
        }

        let mut byte0 = 0x80;
        byte0 |= buttons & 0x07;

        let (x_byte, x_overflow) = clamp_to_i8(dx);
        let (y_byte, y_overflow) = clamp_to_i8(dy);

        if x_overflow {
            byte0 |= 0x10;
        }
        if y_overflow {
            byte0 |= 0x08;
        }
        byte0 |= ((x_byte as u8 >> 7) & 0x01) << 6;
        byte0 |= ((y_byte as u8 >> 7) & 0x01) << 5;

        self.push_output(byte0);
        self.push_output(x_byte as u8);
        self.push_output(y_byte as u8);
    }

    /// Drain pending input events from the input channel.
    fn drain_input_channel(&mut self) {
        let events = if let Some(channel) = &self.input_channel {
            let mut queue = channel.lock().unwrap();
            queue.drain(..).collect::<Vec<_>>()
        } else {
            Vec::new()
        };

        for event in events {
            match event {
                InputEvent::Keyboard { key, pressed } => {
                    self.process_keyboard_event(key, pressed);
                }
                InputEvent::Mouse { buttons, dx, dy } => {
                    self.process_mouse_event(buttons, dx, dy);
                }
            }
        }
    }

    /// Handle a command written to the command port (0x64).
    fn handle_command(&mut self, cmd: u8) {
        match cmd {
            CMD_RESET_CONTROLLER => {
                info!("i8042 reset signalled");
                if let Err(e) = self.reset_evt.write(1) {
                    error!("Error triggering i8042 reset event: {e}");
                }
                while !self.vcpus_kill_signalled.load(Ordering::SeqCst)
                    && !self.vcpus_pause_signalled.load(Ordering::SeqCst)
                {
                    thread::sleep(std::time::Duration::from_millis(1));
                }
            }
            CMD_TEST_CONTROLLER => {
                self.push_output(0x55);
            }
            CMD_SELF_TEST => {
                self.push_output(0x55);
            }
            CMD_DISABLE_KEYBOARD => {
                debug!("i8042: keyboard disabled");
                self.keyboard_enabled = false;
                self.port_a &= !0x01;
                self.push_output(0xFA);
            }
            CMD_ENABLE_KEYBOARD => {
                debug!("i8042: keyboard enabled");
                self.keyboard_enabled = true;
                self.port_a |= 0x01;
                self.push_output(0xFA);
            }
            CMD_DISABLE_SECONDARY => {
                debug!("i8042: mouse/secondary disabled");
                self.mouse_enabled = false;
                self.port_a &= !0x10;
                self.push_output(0xFA);
            }
            CMD_ENABLE_SECONDARY => {
                debug!("i8042: mouse/secondary enabled");
                self.mouse_enabled = true;
                self.port_a |= 0x10;
                self.push_output(0xFA);
            }
            CMD_TEST_KEYBOARD => {
                self.push_output(0xFA);
                self.push_output(0x55);
            }
            CMD_READ_PORT_A => {
                self.push_output(self.port_a);
            }
            CMD_READ_PORT_B => {
                self.push_output(self.port_b);
            }
            CMD_READ_KBD_INPUT => {
                if let Some(byte) = self.output_buffer.pop_front() {
                    self.push_output(byte);
                } else {
                    self.push_output(0x00);
                }
            }
            CMD_WRITE_KBD_OUTPUT => {
                self.pending_command = Some(cmd);
                self.push_output(0xFA);
            }
            CMD_WRITE_SECONDARY_OUTPUT => {
                self.pending_command = Some(cmd);
                self.push_output(0xFA);
            }
            _ => {
                warn!("i8042: unknown command 0x{cmd:02x}");
            }
        }
    }

    /// Handle data written to the data port (0x60).
    fn handle_data_write(&mut self, data: u8) {
        if let Some(cmd) = self.pending_command {
            match cmd {
                CMD_WRITE_KBD_OUTPUT => {
                    if self.keyboard_enabled {
                        self.push_output(data);
                    }
                }
                CMD_WRITE_SECONDARY_OUTPUT => {
                    if self.mouse_enabled {
                        self.push_output(data);
                    }
                }
                _ => {}
            }
            self.pending_command = None;
        } else {
            self.input_buffer = Some(data);
        }
    }
}

/// Clamp a i16 value to i8 range, returning overflow flag.
fn clamp_to_i8(v: i16) -> (i8, bool) {
    if v > i8::MAX as i16 {
        (i8::MAX, true)
    } else if v < i8::MIN as i16 {
        (i8::MIN, true)
    } else {
        (v as i8, false)
    }
}

// i8042 device is located at I/O port 0x60. We implement:
// - Port 0x60 (offset 0): PS/2 data port
// - Port 0x61 (offset 1): Port B register
// - Port 0x62 (offset 2): Timer latch
// - Port 0x63 (offset 3): Mode select
// - Port 0x64 (offset 4): Command/status register
impl BusDevice for I8042Device {
    fn read(&mut self, _base: u64, offset: u64, data: &mut [u8]) {
        if data.len() != 1 {
            warn!("Invalid read size on i8042 device: {}", data.len());
            return;
        }

        match offset {
            OFFSET_DATA => {
                self.drain_input_channel();
                if let Some(byte) = self.output_buffer.pop_front() {
                    data[0] = byte;
                    if self.output_buffer.is_empty() {
                        self.status &= !STATUS_OUTPUT_BUFFER_FULL;
                    }
                } else {
                    data[0] = 0xFF;
                }
            }
            OFFSET_PORT_B => {
                data[0] = self.port_b;
            }
            OFFSET_TIMER_LATCH => {
                data[0] = 0x00;
            }
            OFFSET_MODE_SELECT => {
                data[0] = 0x00;
            }
            OFFSET_COMMAND => {
                data[0] = self.status | STATUS_SYSTEM_FLAG;
            }
            _ => {
                debug!("i8042: read from unknown offset {offset}");
                data[0] = 0x00;
            }
        }
    }

    fn write(&mut self, _base: u64, offset: u64, data: &[u8]) -> Option<Arc<Barrier>> {
        if data.len() != 1 {
            warn!("Invalid write size on i8042 device: {}", data.len());
            return None;
        }

        match offset {
            OFFSET_DATA => {
                self.handle_data_write(data[0]);
            }
            OFFSET_PORT_B => {
                self.port_b = data[0];
            }
            OFFSET_TIMER_LATCH => {
                debug!("i8042: write to timer latch: 0x{:02x}", data[0]);
            }
            OFFSET_MODE_SELECT => {
                debug!("i8042: write to mode select: 0x{:02x}", data[0]);
            }
            OFFSET_COMMAND => {
                self.handle_command(data[0]);
            }
            _ => {
                debug!("i8042: write to unknown offset {}: 0x{:02x}", offset, data[0]);
            }
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_device() -> I8042Device {
        I8042Device {
            reset_evt: EventFd::new(0).unwrap(),
            vcpus_kill_signalled: Arc::new(AtomicBool::new(false)),
            vcpus_pause_signalled: Arc::new(AtomicBool::new(false)),
            status: STATUS_SYSTEM_FLAG,
            output_buffer: VecDeque::new(),
            input_buffer: None,
            port_a: 0x00,
            port_b: 0x20,
            keyboard_enabled: true,
            mouse_enabled: true,
            pending_command: None,
            input_channel: None,
        }
    }

    #[test]
    fn test_port_b_read() {
        let mut dev = make_device();
        let mut data = [0u8];
        dev.read(0, OFFSET_PORT_B, &mut data);
        assert_eq!(data[0], 0x20);
    }

    #[test]
    fn test_port_b_write() {
        let mut dev = make_device();
        dev.write(0, OFFSET_PORT_B, &[0x40]);
        let mut data = [0u8];
        dev.read(0, OFFSET_PORT_B, &mut data);
        assert_eq!(data[0], 0x40);
    }

    #[test]
    fn test_status_read() {
        let mut dev = make_device();
        let mut data = [0u8];
        dev.read(0, OFFSET_COMMAND, &mut data);
        assert!(data[0] & STATUS_SYSTEM_FLAG != 0);
    }

    #[test]
    fn test_keyboard_enable_disable() {
        let mut dev = make_device();

        dev.write(0, OFFSET_COMMAND, &[CMD_DISABLE_KEYBOARD]);
        assert!(!dev.keyboard_enabled);
        assert_eq!(dev.port_a & 0x01, 0);

        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0xFA);

        dev.write(0, OFFSET_COMMAND, &[CMD_ENABLE_KEYBOARD]);
        assert!(dev.keyboard_enabled);
        assert_eq!(dev.port_a & 0x01, 0x01);
    }

    #[test]
    fn test_mouse_enable_disable() {
        let mut dev = make_device();

        dev.write(0, OFFSET_COMMAND, &[CMD_DISABLE_SECONDARY]);
        assert!(!dev.mouse_enabled);
        assert_eq!(dev.port_a & 0x10, 0);

        dev.write(0, OFFSET_COMMAND, &[CMD_ENABLE_SECONDARY]);
        assert!(dev.mouse_enabled);
        assert_eq!(dev.port_a & 0x10, 0x10);
    }

    #[test]
    fn test_test_controller() {
        let mut dev = make_device();
        dev.write(0, OFFSET_COMMAND, &[CMD_TEST_CONTROLLER]);

        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0x55);
    }

    #[test]
    fn test_read_port_a() {
        let mut dev = make_device();
        dev.port_a = 0xAB;

        dev.write(0, OFFSET_COMMAND, &[CMD_READ_PORT_A]);
        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0xAB);
    }

    #[test]
    fn test_read_port_b() {
        let mut dev = make_device();
        dev.port_b = 0x42;

        dev.write(0, OFFSET_COMMAND, &[CMD_READ_PORT_B]);
        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0x42);
    }

    #[test]
    fn test_keyboard_scan_codes() {
        let mut dev = make_device();

        dev.process_keyboard_event(0x10, true);
        assert_eq!(dev.output_buffer.len(), 1);
        assert_eq!(dev.output_buffer[0], 0x10);

        dev.process_keyboard_event(0x10, false);
        assert_eq!(dev.output_buffer.len(), 3);
        assert_eq!(dev.output_buffer[1], 0xF0);
        assert_eq!(dev.output_buffer[2], 0x10);
    }

    #[test]
    fn test_keyboard_disabled() {
        let mut dev = make_device();
        dev.keyboard_enabled = false;

        dev.process_keyboard_event(0x10, true);
        assert_eq!(dev.output_buffer.len(), 0);
    }

    #[test]
    fn test_mouse_packet() {
        let mut dev = make_device();

        dev.process_mouse_event(0x01, 10, -5);

        assert_eq!(dev.output_buffer.len(), 3);
        let packet = dev.output_buffer.make_contiguous();
        assert_eq!(packet[0] & 0x80, 0x80);
        assert_eq!(packet[0] & 0x01, 0x01);
        assert_eq!(packet[1], 10);
        assert_eq!(packet[2], (-5i8) as u8);
    }

    #[test]
    fn test_mouse_overflow() {
        let mut dev = make_device();

        dev.process_mouse_event(0x00, 300, 0);

        assert_eq!(dev.output_buffer.len(), 3);
        let packet = dev.output_buffer.make_contiguous();
        assert_ne!(packet[0] & 0x10, 0);
        assert_eq!(packet[1], i8::MAX as u8);
    }

    #[test]
    fn test_mouse_disabled() {
        let mut dev = make_device();
        dev.mouse_enabled = false;

        dev.process_mouse_event(0x01, 10, 5);
        assert_eq!(dev.output_buffer.len(), 0);
    }

    #[test]
    fn test_input_channel() {
        let channel = Arc::new(Mutex::new(VecDeque::new()));
        let mut dev = I8042Device {
            input_channel: Some(channel.clone()),
            ..make_device()
        };

        channel.lock().unwrap().push_back(InputEvent::Keyboard {
            key: 0x10,
            pressed: true,
        });

        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0x10);
    }

    #[test]
    fn test_data_port_empty_read() {
        let mut dev = make_device();
        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0xFF);
    }

    #[test]
    fn test_write_keyboard_output() {
        let mut dev = make_device();

        dev.write(0, OFFSET_COMMAND, &[CMD_WRITE_KBD_OUTPUT]);
        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0xFA);

        dev.write(0, OFFSET_DATA, &[0x55]);
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0x55);
    }

    #[test]
    fn test_write_secondary_output() {
        let mut dev = make_device();

        dev.write(0, OFFSET_COMMAND, &[CMD_WRITE_SECONDARY_OUTPUT]);
        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0xFA);

        dev.write(0, OFFSET_DATA, &[0xAA]);
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0xAA);
    }

    #[test]
    fn test_output_buffer_full() {
        let mut dev = make_device();
        for _ in 0..MAX_DATA_BUFFER {
            dev.push_output(0x42);
        }
        assert_eq!(dev.output_buffer.len(), MAX_DATA_BUFFER);

        dev.push_output(0xFF);
        assert_eq!(dev.output_buffer.len(), MAX_DATA_BUFFER);
    }

    #[test]
    fn test_clamp_to_i8() {
        assert_eq!(clamp_to_i8(0), (0, false));
        assert_eq!(clamp_to_i8(127), (127, false));
        assert_eq!(clamp_to_i8(128), (127, true));
        assert_eq!(clamp_to_i8(-128), (-128, false));
        assert_eq!(clamp_to_i8(-129), (-128, true));
        assert_eq!(clamp_to_i8(32767), (127, true));
        assert_eq!(clamp_to_i8(-32768), (-128, true));
    }
}
