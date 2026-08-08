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
use vm_device::interrupt::InterruptSourceGroup;
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

// CTR (Controller Configuration Register) bits
const CTR_XLATE: u8 = 1 << 6; // Scancode translation enable (Set 2 → Set 1)
const CMD_WRITE_PORT_A: u8 = 0x60;
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
const CMD_PULSE_INTERRUPT: u8 = 0xA9;
const CMD_READ_KBD_CTRL: u8 = 0xAB;
const CMD_WRITE_KBD_CTRL: u8 = 0xB0;

// PS/2 status register bits
const STATUS_OUTPUT_BUFFER_FULL: u8 = 1 << 0;
#[allow(dead_code)]
const STATUS_INPUT_BUFFER_FULL: u8 = 1 << 1;
const STATUS_SYSTEM_FLAG: u8 = 1 << 2;
#[allow(dead_code)]
const STATUS_COMMAND_DATA: u8 = 1 << 3;
const STATUS_KEYLOCK: u8 = 1 << 4;
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
    /// Generate PS/2 Set 1 scan codes from an X11 keysym (US keyboard layout).
    /// Returns a Vec of bytes to send to the PS/2 data port.
    ///
    /// Returns (make_code, needs_e0_prefix). Keys in the dedicated arrow/navigation
    /// cluster and KP_Enter/KP_Divide require the 0xE0 extension prefix.
    fn scan_codes(key: u32, pressed: bool) -> Vec<u8> {
        let (make_code, e0) = match key {
            // Letters (lowercase X11 keysyms) -> PS/2 Set 1
            0x61 => (0x1E, false), // a
            0x62 => (0x30, false), // b
            0x63 => (0x2E, false), // c
            0x64 => (0x20, false), // d
            0x65 => (0x12, false), // e
            0x66 => (0x21, false), // f
            0x67 => (0x22, false), // g
            0x68 => (0x23, false), // h
            0x69 => (0x17, false), // i
            0x6A => (0x24, false), // j
            0x6B => (0x25, false), // k
            0x6C => (0x26, false), // l
            0x6D => (0x32, false), // m
            0x6E => (0x31, false), // n
            0x6F => (0x18, false), // o
            0x70 => (0x19, false), // p
            0x71 => (0x10, false), // q
            0x72 => (0x13, false), // r
            0x73 => (0x1F, false), // s
            0x74 => (0x14, false), // t
            0x75 => (0x16, false), // u
            0x76 => (0x2F, false), // v
            0x77 => (0x11, false), // w
            0x78 => (0x2D, false), // x
            0x79 => (0x15, false), // y
            0x7A => (0x2C, false), // z

            // Uppercase letters (same scan codes, shift handled by guest OS)
            0x41 => (0x1E, false), // A
            0x42 => (0x30, false), // B
            0x43 => (0x2E, false), // C
            0x44 => (0x20, false), // D
            0x45 => (0x12, false), // E
            0x46 => (0x21, false), // F
            0x47 => (0x22, false), // G
            0x48 => (0x23, false), // H
            0x49 => (0x17, false), // I
            0x4A => (0x24, false), // J
            0x4B => (0x25, false), // K
            0x4C => (0x26, false), // L
            0x4D => (0x32, false), // M
            0x4E => (0x31, false), // N
            0x4F => (0x18, false), // O
            0x50 => (0x19, false), // P
            0x51 => (0x10, false), // Q
            0x52 => (0x13, false), // R
            0x53 => (0x1F, false), // S
            0x54 => (0x14, false), // T
            0x55 => (0x16, false), // U
            0x56 => (0x2F, false), // V
            0x57 => (0x11, false), // W
            0x58 => (0x2D, false), // X
            0x59 => (0x15, false), // Y
            0x5A => (0x2C, false), // Z

            // Digits
            0x30 => (0x0B, false), // 0
            0x31 => (0x02, false), // 1
            0x32 => (0x03, false), // 2
            0x33 => (0x04, false), // 3
            0x34 => (0x05, false), // 4
            0x35 => (0x06, false), // 5
            0x36 => (0x07, false), // 6
            0x37 => (0x08, false), // 7
            0x38 => (0x09, false), // 8
            0x39 => (0x0A, false), // 9

            // Punctuation / symbols
            0x20 => (0x39, false), // Space
            0x60 => (0x29, false), // Grave (`)
            0x2D => (0x0C, false), // Minus (-)
            0x3D => (0x0D, false), // Equals (=)
            0x5B => (0x1A, false), // Left Bracket ([)
            0x5D => (0x1B, false), // Right Bracket (])
            0x5C => (0x2B, false), // Backslash (\)
            0x3B => (0x27, false), // Semicolon (;)
            0x27 => (0x28, false), // Quote (')
            0x2C => (0x33, false), // Comma (,)
            0x2E => (0x34, false), // Period (.)
            0x2F => (0x35, false), // Slash (/)

            // Control keys
            0xFF1B => (0x01, false), // Escape
            0xFF09 => (0x0F, false), // Tab
            0xFF0D => (0x1C, false), // Return/Enter
            0xFFE1 => (0x2A, false), // Shift_L
            0xFFE2 => (0x36, false), // Shift_R
            0xFFE3 => (0x1D, false), // Control_L
            0xFFE4 => (0x1D, false), // Control_R
            0xFFE9 => (0x38, false), // Alt_L
            0xFFEA => (0xB8, false), // Alt_R (AltGr)
            0xFFE5 => (0x3A, false), // Caps_Lock
            0xFF08 => (0x0E, false), // BackSpace

            // Dedicated navigation cluster (E0-prefixed in Set 1)
            0xFFFF => (0x53, true), // Delete
            0xFF67 => (0x53, true), // Delete (alternate)
            0xFF50 => (0x47, true), // Home
            0xFF57 => (0x4F, true), // End
            0xFF55 => (0x49, true), // Prior/PageUp
            0xFF56 => (0x51, true), // Next/PageDown
            0xFF52 => (0x48, true), // Up
            0xFF54 => (0x50, true), // Down
            0xFF51 => (0x4B, true), // Left
            0xFF53 => (0x4D, true), // Right
            0xFF63 => (0x52, true), // Insert

            // F keys
            0xFFBE => (0x3B, false), // F1
            0xFFBF => (0x3C, false), // F2
            0xFFC0 => (0x3D, false), // F3
            0xFFC1 => (0x3E, false), // F4
            0xFFC2 => (0x3F, false), // F5
            0xFFC3 => (0x40, false), // F6
            0xFFC4 => (0x41, false), // F7
            0xFFC5 => (0x42, false), // F8
            0xFFC6 => (0x43, false), // F9
            0xFFC7 => (0x44, false), // F10
            0xFFC8 => (0x45, false), // F11
            0xFFC9 => (0x46, false), // F12

            // Numpad (Set 1 codes, share with navigation keys)
            0xFF90 => (0x52, false), // KP_0
            0xFF91 => (0x4F, false), // KP_1
            0xFF92 => (0x50, false), // KP_2
            0xFF93 => (0x51, false), // KP_3
            0xFF94 => (0x4B, false), // KP_4
            0xFF95 => (0x4C, false), // KP_5
            0xFF96 => (0x4D, false), // KP_6
            0xFF97 => (0x47, false), // KP_7
            0xFF98 => (0x48, false), // KP_8
            0xFF99 => (0x49, false), // KP_9
            0xFFAE => (0x53, false), // KP_Decimal
            0xFF8D => (0x1C, true), // KP_Enter (E0-prefixed)
            0xFF6B => (0x4E, false), // KP_Add
            0xFF6D => (0x4A, false), // KP_Subtract
            0xFF6A => (0x37, false), // KP_Multiply
            0xFF6F => (0x35, true), // KP_Divide (E0-prefixed)
            _ => (0x00, false),      // Unknown key
        };

        if make_code == 0 {
            return Vec::new();
        }

        if pressed {
            if e0 {
                vec![0xE0, make_code]
            } else {
                vec![make_code]
            }
        } else {
            if e0 {
                vec![0xE0, 0xF0, make_code]
            } else {
                vec![0xF0, make_code]
            }
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

    // Controller configuration register (CTR)
    // Bit 1: KBD INT (keyboard interrupt enable)
    // Bit 2: SYS FLAG (system flag)
    // Bit 3: KBD DIS (keyboard disable)
    // Bit 4: AUX DIS (auxiliary disable)
    // Bit 5: AUX INT (auxiliary interrupt enable)
    // Bit 6: XLATE (scancode translation: 1 = Set 2→Set 1, 0 = raw Set 2)
    // Bit 7: CPU INT (CPU interrupt enable)
    ctr: u8,

    // Control register (bit 6 = keylock, must be 0 for atkbd to load)
    control_reg: u8,

    // Keyboard state
    keyboard_enabled: bool,
    // Mouse (secondary) state
    mouse_enabled: bool,

    // Pending command that needs data from input buffer
    pending_command: Option<u8>,

    // Channel to receive input events from VNC or other sources
    input_channel: Option<Arc<Mutex<VecDeque<InputEvent>>>>,

    // IRQ for keyboard (primary PS/2) interrupt
    irq: Option<Arc<dyn InterruptSourceGroup>>,
}

impl I8042Device {
    /// Constructs a i8042 device.
    ///
    /// `reset_evt` - EventFd to signal on guest reset request.
    /// `vcpus_kill_signalled` - Flag indicating vCPUs have been signaled to stop.
    /// `vcpus_pause_signalled` - Flag indicating vCPUs have been signaled to pause.
    /// `input_channel` - Optional shared queue for receiving input events from VNC.
    /// `irq` - Optional IRQ source for keyboard interrupt.
    pub fn new(
        reset_evt: EventFd,
        vcpus_kill_signalled: Arc<AtomicBool>,
        vcpus_pause_signalled: Arc<AtomicBool>,
        input_channel: Option<Arc<Mutex<VecDeque<InputEvent>>>>,
        irq: Option<Arc<dyn InterruptSourceGroup>>,
    ) -> I8042Device {
        I8042Device {
            reset_evt,
            vcpus_kill_signalled,
            vcpus_pause_signalled,
            status: STATUS_SYSTEM_FLAG | STATUS_KEYLOCK,
            output_buffer: VecDeque::new(),
            input_buffer: None,
            port_a: 0x00,
            port_b: 0x23,
            ctr: CTR_XLATE, // Translated mode: i8042 converts Set 2 → Set 1
            control_reg: 0x00,
            keyboard_enabled: true,
            mouse_enabled: true,
            pending_command: None,
            input_channel,
            irq,
        }
    }

    /// Get a clone of the IRQ source group, if present.
    pub fn irq(&self) -> Option<Arc<dyn InterruptSourceGroup>> {
        self.irq.as_ref().map(Arc::clone)
    }

    /// Check whether the output buffer is nearly full.
    /// Returns true when the buffer has fewer than 4 free slots,
    /// giving the caller a chance to back off before events are dropped.
    pub fn output_buffer_near_full(&self) -> bool {
        self.output_buffer.len() >= MAX_DATA_BUFFER - 4
    }

    /// Push a byte to the output buffer and trigger interrupt if data was added.
    fn push_output(&mut self, byte: u8) {
        if self.output_buffer.len() < MAX_DATA_BUFFER {
            // Only trigger IRQ when buffer transitions from empty to non-empty
            let was_empty = self.output_buffer.is_empty();
            self.output_buffer.push_back(byte);
            self.status |= STATUS_OUTPUT_BUFFER_FULL;
            if was_empty {
                if let Some(ref irq) = self.irq {
                    if let Err(e) = irq.trigger(0) {
                        warn!("i8042: failed to trigger IRQ: {e}");
                    }
                }
            }
        } else {
            debug!("i8042: output buffer full, dropping byte 0x{byte:02x}");
        }
    }

    /// Process a keyboard input event and generate scan codes.
    /// Returns true if scan codes were added to the output buffer.
    pub fn process_keyboard_event(&mut self, key: u32, pressed: bool) -> bool {
        if !self.keyboard_enabled {
            return false;
        }
        let scan_codes = KeyboardMap::scan_codes(key, pressed);
        if !scan_codes.is_empty() {
            info!(
                "i8042: keysym=0x{:x} pressed={} -> scan_codes=[{:02x?}]",
                key, pressed, scan_codes
            );
            self.push_output_bytes_no_irq(&scan_codes);
            true
        } else {
            info!("i8042: unknown keysym=0x{:x} pressed={}", key, pressed);
            false
        }
    }

    /// Push bytes to output buffer without triggering IRQ.
    fn push_output_bytes_no_irq(&mut self, bytes: &[u8]) {
        for &b in bytes {
            if self.output_buffer.len() < MAX_DATA_BUFFER {
                self.output_buffer.push_back(b);
                self.status |= STATUS_OUTPUT_BUFFER_FULL;
            }
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
        debug!("i8042: cmd write 0x{cmd:02x}");
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
                // Linux uses 0x20 to read the CTR during i8042 initialization.
                // It checks the XLATE bit (0x40) to determine whether the controller
                // translates Set 2 → Set 1 scancodes. If XLATE is clear, atkbd uses
                // the Set 2 keycode table, causing Set 1 scancodes to be misinterpreted.
                self.push_output(self.ctr);
            }
            CMD_WRITE_PORT_A => {
                self.pending_command = Some(cmd);
                self.push_output(0xFA);
            }
            CMD_PULSE_INTERRUPT => {
                self.push_output(0xFA);
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
            CMD_READ_KBD_CTRL => {
                self.push_output(self.control_reg);
            }
            CMD_WRITE_KBD_CTRL => {
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
        debug!("i8042: data write 0x{data:02x}");
        if let Some(cmd) = self.pending_command {
            match cmd {
                CMD_WRITE_KBD_OUTPUT => {
                    // Emulate keyboard response for GET_ID (0xF2)
                    if data == 0xF2 {
                        self.push_output(0xFA);
                        self.push_output(0xAB);
                        self.push_output(0x83);
                    }
                }
                CMD_WRITE_SECONDARY_OUTPUT => {
                    // Forward data to mouse/secondary interface (no echo to output buffer)
                }
                CMD_WRITE_PORT_A => {
                    self.port_a = data;
                }
                CMD_WRITE_KBD_CTRL => {
                    // Bit 6 = keylock, bit 1 = translate, bit 2/3 = IRQ enable
                    self.control_reg = data;
                }
                _ => {}
            }
            self.pending_command = None;
        } else {
            // Direct keyboard command (e.g., 0xF2 = GET_ID, 0xED = SET LED)
            if data == 0xF2 && self.keyboard_enabled {
                // Emulate AT keyboard GET_ID response
                self.push_output(0xFA);
                self.push_output(0xAB);
                self.push_output(0x83);
            } else if data == 0xED && self.keyboard_enabled {
                // SET LED command - acknowledge, parameter follows
                self.push_output(0xFA);
            } else if data == 0xF4 {
                // ENABLE typing - acknowledge and enable keyboard data
                debug!("i8042: keyboard enabled (0xF4)");
                self.keyboard_enabled = true;
                self.port_a |= 0x01;
                self.push_output(0xFA);
            } else if data == 0xF5 {
                // DISABLE typing (reset to defaults and disable) - acknowledge
                debug!("i8042: keyboard disabled (0xF5)");
                self.keyboard_enabled = false;
                self.port_a &= !0x01;
                self.push_output(0xFA);
            } else {
                self.input_buffer = Some(data);
            }
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
                    debug!("i8042: guest read 0x{byte:02x}");
                    if self.output_buffer.is_empty() {
                        self.status &= !STATUS_OUTPUT_BUFFER_FULL;
                    } else {
                        // More data available, trigger another IRQ
                        if let Some(ref irq) = self.irq {
                            let _ = irq.trigger(0);
                        }
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
                data[0] = self.status | STATUS_SYSTEM_FLAG | STATUS_KEYLOCK;
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
            status: STATUS_SYSTEM_FLAG | STATUS_KEYLOCK,
            output_buffer: VecDeque::new(),
            input_buffer: None,
            port_a: 0x00,
            port_b: 0x20,
            ctr: CTR_XLATE,
            control_reg: 0x00,
            keyboard_enabled: true,
            mouse_enabled: true,
            pending_command: None,
            input_channel: None,
            irq: None,
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
    fn test_keyboard_enable_disable_data_port() {
        // Test 0xF5 (DISABLE) and 0xF4 (ENABLE) written directly to data port.
        // These are the commands the Linux atkbd driver sends during init:
        // atkbd_deactivate() sends 0xF5, atkbd_activate() sends 0xF4.
        let mut dev = make_device();

        // 0xF5 = DISABLE typing (atkbd_deactivate)
        dev.write(0, OFFSET_DATA, &[0xF5]);
        assert!(!dev.keyboard_enabled);
        assert_eq!(dev.port_a & 0x01, 0);

        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0xFA); // keyboard ACK

        // 0xF4 = ENABLE typing (atkbd_activate)
        dev.write(0, OFFSET_DATA, &[0xF4]);
        assert!(dev.keyboard_enabled);
        assert_eq!(dev.port_a & 0x01, 0x01);

        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0xFA); // keyboard ACK
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
        dev.ctr = 0xAB;

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

        // X11 keysym 0x71 ('q') -> PS/2 scan code 0x10
        dev.process_keyboard_event(0x71, true);
        assert_eq!(dev.output_buffer.len(), 1);
        assert_eq!(dev.output_buffer[0], 0x10);

        dev.process_keyboard_event(0x71, false);
        assert_eq!(dev.output_buffer.len(), 3);
        assert_eq!(dev.output_buffer[1], 0xF0);
        assert_eq!(dev.output_buffer[2], 0x10);
    }

    #[test]
    fn test_keyboard_disabled() {
        let mut dev = make_device();
        dev.keyboard_enabled = false;

        dev.process_keyboard_event(0x71, true); // X11 keysym 'q'
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

        // X11 keysym 0x71 ('q') -> PS/2 scan code 0x10
        channel.lock().unwrap().push_back(InputEvent::Keyboard {
            key: 0x71,
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

        // Data is forwarded to keyboard interface, not echoed to output buffer
        dev.write(0, OFFSET_DATA, &[0x55]);
        assert_eq!(dev.output_buffer.len(), 0);
    }

    #[test]
    fn test_write_secondary_output() {
        let mut dev = make_device();

        dev.write(0, OFFSET_COMMAND, &[CMD_WRITE_SECONDARY_OUTPUT]);
        let mut data = [0u8];
        dev.read(0, OFFSET_DATA, &mut data);
        assert_eq!(data[0], 0xFA);

        // Data is forwarded to mouse interface, not echoed to output buffer
        dev.write(0, OFFSET_DATA, &[0xAA]);
        assert_eq!(dev.output_buffer.len(), 0);
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
