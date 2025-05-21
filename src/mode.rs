#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum OperatingMode {
    Config = 0x00,
    NDOF = 0x0C,
}
