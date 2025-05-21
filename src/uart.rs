// src/uart.rs
#![cfg(feature = "uart")]
use embedded_io_async::{Read, Write};
use crate::regs::*;
use crate::types::*;
use defmt::Format;
use embassy_time::Duration;
use embassy_time::with_timeout;

#[derive(Debug)]
#[derive(Format)]
pub enum Error<E = ()> {
    UartWrite,
    UartRead,
    UnexpectedResponse,
    Transport(E), // for underlying UART errors if needed
}

pub struct Bno055Uart<UART>
where
    UART: Read + Write + Unpin,
{
    uart: UART,
    pub mode: BNO055OperationMode,
}

impl<UART> Bno055Uart<UART>
where
    UART: Read + Write + Unpin,
{
    pub fn new(uart: UART) -> Self {
        Self {
            uart,
            mode: BNO055OperationMode::CONFIG_MODE,
        }
    }

    pub async fn init(&mut self) -> Result<(), Error> {
        self.write_register(BNO055_OPR_MODE, BNO055OperationMode::CONFIG_MODE.bits()).await?;
        embassy_time::Timer::after_millis(25).await;

        self.write_register(BNO055_PAGE_ID, 0x00).await?;
        self.write_register(BNO055_OPR_MODE, BNO055OperationMode::NDOF.bits()).await?;
        embassy_time::Timer::after_millis(600).await;

        Ok(())
    }

    pub async fn write_register(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        let packet = [0xAA, 0x00, reg, 1, val];
        let checksum = calc_checksum(&packet[1..]);

        let mut frame = [0u8; 6];
        frame[..5].copy_from_slice(&packet);
        frame[5] = checksum;

        // Write each byte individually and wait 2ms between each write as the BNO055
        // datasheet recommends a 2ms delay between each byte.
        // This is a workaround for the UART not being able to handle the full frame at once.
        // This is not ideal, but it works for now.
        for byte in &frame {
            let written = self.uart.write(core::slice::from_ref(byte)).await.map_err(|_| Error::UartWrite)?;
            if written != 1 {
                return Err(Error::UartWrite);
            }
            embassy_time::Timer::after_millis(2).await;
        }
        Ok(())
    }

    pub async fn read_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error> {
        let req = [0xAA, 0x01, reg, buf.len() as u8];
        self.uart.write_all(&req).await.map_err(|_| Error::UartWrite)?;

        // Timeout duration (adjust as needed)
        let timeout = Duration::from_millis(100);

        // Read header with timeout
        let mut header = [0u8; 2];
        with_timeout(timeout, self.uart.read_exact(&mut header))
            .await
            .map_err(|_| Error::UartRead)?
            .map_err(|_| Error::UartRead)?;

        if header[0] != 0xBB {
            return Err(Error::UnexpectedResponse);
        }
        if header[1] != buf.len() as u8 {
            return Err(Error::UnexpectedResponse);
        }

        // Read payload with timeout
        with_timeout(timeout, self.uart.read_exact(buf))
            .await
            .map_err(|_| Error::UartRead)?
            .map_err(|_| Error::UartRead)?;

        Ok(())
    }

    /// Performs a soft reset, reinitializing the BNO055 register map.
    /// Refer to section 3.2 in the datasheet.
    pub async fn soft_reset(&mut self) -> Result<(), Error> {
        self.write_register(BNO055_SYS_TRIGGER, BNO055_SYS_TRIGGER_RST_SYS_BIT).await?;

        // Wait for reset and bootup (datasheet recommends ~650ms)
        embassy_time::Timer::after_millis(650).await;

        Ok(())
    }

    pub async fn set_mode(&mut self, mode: BNO055OperationMode) -> Result<(), Error> {
        if self.mode != mode {
            self.write_register(BNO055_OPR_MODE, mode.bits()).await?;
            embassy_time::Timer::after_millis(19).await;
            self.mode = mode;
        }
        Ok(())
    }

    /// Reads the current power mode (Normal, LowPower, Suspend)
    pub async fn power_mode(&mut self) -> Result<BNO055PowerMode, Error> {
        let mut buf = [0u8; 1];
        self.read_register(BNO055_PWR_MODE, &mut buf).await?;

        Ok(BNO055PowerMode::from_bits_truncate(buf[0]))
    }

    /// Get calibration status
    pub async fn get_calibration_status(&mut self) -> Result<BNO055CalibrationStatus, Error> {
        let mut buf = [0u8; 1];
        self.read_register(BNO055_CALIB_STAT, &mut buf).await?;

        let calib = buf[0];
        let sys = (calib >> 6) & 0x03;
        let gyr = (calib >> 4) & 0x03;
        let acc = (calib >> 2) & 0x03;
        let mag = calib & 0x03;

        Ok(BNO055CalibrationStatus { sys, gyr, acc, mag })
    }

    /// Checks whether device is fully calibrated or not.
    pub async fn is_fully_calibrated(&mut self) -> Result<bool, Error> {
        let status = self.get_calibration_status().await?;
        Ok(status.mag == 3 && status.gyr == 3 && status.acc == 3 && status.sys == 3)
    }

    pub async fn quaternion(&mut self) -> Result<Quaternion, Error> {
        const LEN: usize = 8;
        let mut raw = [0u8; LEN];
        self.read_register(BNO055_QUA_DATA_W_LSB, &mut raw).await?;

        let w = i16::from_le_bytes([raw[0], raw[1]]) as f32 / 16384.0;
        let x = i16::from_le_bytes([raw[2], raw[3]]) as f32 / 16384.0;
        let y = i16::from_le_bytes([raw[4], raw[5]]) as f32 / 16384.0;
        let z = i16::from_le_bytes([raw[6], raw[7]]) as f32 / 16384.0;

        Ok(Quaternion {
            s: w,
            v: [x, y, z],
        })
    }

}

fn calc_checksum(data: &[u8]) -> u8 {
    data.iter().fold(0, |sum, b| sum.wrapping_add(*b))
}
