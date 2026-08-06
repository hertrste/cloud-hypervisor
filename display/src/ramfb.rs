use std::mem::size_of;

/// RAMFB_CONFIG as defined by EDK2 QemuRamfbDxe.
/// All multi-byte fields are written in big-endian by the firmware.
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct RamfbConfig {
    pub address: u64,
    pub fourcc: u32,
    pub flags: u32,
    pub width: u32,
    pub height: u32,
    pub stride: u32,
}

/// Size of RAMFB_CONFIG in bytes
pub const RAMFB_CONFIG_SIZE: usize = size_of::<RamfbConfig>();

/// DRM_FORMAT_XRGB8888 (0x34325258)
pub const DRM_FORMAT_XRGB8888: u32 = 0x34325258;

impl RamfbConfig {
    /// Create a new RamfbConfig from big-endian bytes written by the firmware.
    pub fn from_be_bytes(bytes: &[u8; RAMFB_CONFIG_SIZE]) -> Self {
        Self {
            address: u64::from_be_bytes([
                bytes[0], bytes[1], bytes[2], bytes[3],
                bytes[4], bytes[5], bytes[6], bytes[7],
            ]),
            fourcc: u32::from_be_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]),
            flags: u32::from_be_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]),
            width: u32::from_be_bytes([bytes[16], bytes[17], bytes[18], bytes[19]]),
            height: u32::from_be_bytes([bytes[20], bytes[21], bytes[22], bytes[23]]),
            stride: u32::from_be_bytes([bytes[24], bytes[25], bytes[26], bytes[27]]),
        }
    }

    /// Returns the framebuffer size in bytes.
    pub fn framebuffer_size(&self) -> u64 {
        self.stride as u64 * self.height as u64
    }
}
