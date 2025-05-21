#[derive(Debug)]
pub enum Error {
    I2c,
    InvalidChipId(u8),
}
