use std::sync::Mutex;

use log::{debug, error, info};
use vm_memory::{
    Address, GuestAddress, GuestMemoryAtomic, GuestMemoryMmap, GuestAddressSpace, Bytes,
};
use vm_memory::bitmap::AtomicBitmap;

use crate::ramfb::{RamfbConfig, DRM_FORMAT_XRGB8888};

/// Inner state protected by a single mutex to prevent config/initialized races.
struct Inner {
    config: RamfbConfig,
    initialized: bool,
}

/// Framebuffer surface that reads from guest memory.
pub struct FramebufferSurface {
    guest_memory: GuestMemoryAtomic<GuestMemoryMmap<AtomicBitmap>>,
    inner: Mutex<Inner>,
}

impl FramebufferSurface {
    pub fn new(guest_memory: GuestMemoryAtomic<GuestMemoryMmap<AtomicBitmap>>) -> Self {
        Self {
            guest_memory,
            inner: Mutex::new(Inner {
                config: RamfbConfig::default(),
                initialized: false,
            }),
        }
    }

    /// Apply a new framebuffer configuration from the firmware.
    pub fn set_config(&self, config: RamfbConfig) {
        let c_addr = config.address;
        let c_width = config.width;
        let c_height = config.height;
        let c_stride = config.stride;
        let c_fourcc = config.fourcc;

        info!(
            "ramfb: framebuffer configured at GPA 0x{:x}, {}x{}, stride={}, format=0x{:08X}",
            c_addr, c_width, c_height, c_stride, c_fourcc
        );

        if c_fourcc != DRM_FORMAT_XRGB8888 {
            error!(
                "ramfb: unsupported pixel format 0x{:08X}, expected XRGB8888",
                c_fourcc
            );
            return;
        }

        let fb_size = config.framebuffer_size();
        let fb_start = GuestAddress(c_addr);
        let _fb_end = fb_start.checked_add(fb_size).expect("Framebuffer address overflow");

        let mut inner = self.inner.lock().unwrap();
        inner.config = config;
        inner.initialized = true;
        info!(
            "ramfb: framebuffer ready at 0x{:x}, {} bytes",
            c_addr, fb_size
        );
    }

    /// Check if the framebuffer has been initialized by the firmware.
    pub fn is_initialized(&self) -> bool {
        self.inner.lock().unwrap().initialized
    }

    /// Get the current framebuffer configuration.
    pub fn config(&self) -> RamfbConfig {
        self.inner.lock().unwrap().config
    }

    /// Read the entire framebuffer into a buffer.
    /// Returns bytes in XRGB8888 format (4 bytes per pixel).
    pub fn read_framebuffer(&self) -> Option<Vec<u8>> {
        let inner = self.inner.lock().unwrap();
        if !inner.initialized {
            return None;
        }

        let c_addr = inner.config.address;
        let c_width = inner.config.width;
        let c_height = inner.config.height;
        let c_stride = inner.config.stride;
        let fb_size = inner.config.framebuffer_size() as usize;
        debug!(
            "ramfb: read_framebuffer addr=0x{:x} size={} ({}x{}, stride={})",
            c_addr, fb_size, c_width, c_height, c_stride
        );
        let mut data = vec![0u8; fb_size];
        let gm = self.guest_memory.memory();
        let fb_start = GuestAddress(c_addr);

        match gm.read(&mut data, fb_start) {
            Ok(_) => Some(data),
            Err(e) => {
                error!("ramfb: failed to read framebuffer at 0x{:x}: {e}", c_addr);
                None
            }
        }
    }

    /// Read a rectangular region of the framebuffer.
    /// Returns bytes in XRGB8888 format.
    pub fn read_region(&self, x: u32, y: u32, width: u32, height: u32) -> Option<Vec<u8>> {
        let inner = self.inner.lock().unwrap();
        if !inner.initialized {
            return None;
        }

        let cfg_width = inner.config.width;
        let cfg_height = inner.config.height;
        if x + width > cfg_width || y + height > cfg_height {
            debug!(
                "ramfb: region ({},{}) {}x{} exceeds framebuffer {}x{}",
                x, y, width, height, cfg_width, cfg_height
            );
            return None;
        }

        let c_stride = inner.config.stride;
        let c_addr = inner.config.address;
        let bytes_per_pixel = 4u64;
        let region_size = (width as u64 * bytes_per_pixel * height as u64) as usize;
        let offset = (y as u64 * c_stride as u64 + x as u64 * bytes_per_pixel) as usize;

        let gm = self.guest_memory.memory();
        let fb_start = GuestAddress(c_addr);

        let mut result = Vec::with_capacity(region_size);
        for row in 0..height {
            let row_offset = offset + (row as u64 * c_stride as u64) as usize;
            let row_data_start = fb_start.checked_add(row_offset as u64)?;
            let row_len = width as usize * bytes_per_pixel as usize;
            let mut row_buf = vec![0u8; row_len];

            if gm.read(&mut row_buf, row_data_start).is_err() {
                return None;
            }
            result.extend_from_slice(&row_buf);
        }

        Some(result)
    }
}
