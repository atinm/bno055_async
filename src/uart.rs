// uart.rs

use embedded_io_async::{Read, Write};
use embassy_time::{Duration, with_timeout, Timer};
use crate::regs::*;
use crate::types::*;

#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Bno055Uart<UART>
where
    UART: Read + Write + Unpin,
{
    uart: UART,
    pub mode: BNO055OperationMode,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Error<E = ()> {
    UartWrite,
    UartRead,
    UnexpectedWriteResponse,
    UnexpectedReadResponse,
    UnexpectedReadLengthResponse,
    ResetFailed,
    Timeout,
    AckError(u8),
    Transport(E), // for underlying UART errors if needed
}

const UART_READ_TIMEOUT: Duration = Duration::from_millis(50);

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
        defmt::info!("Doing Soft reset");
        self.reset().await?;
        defmt::info!("Done Soft reset");
        // read the chip ID
        let mut buf = [0u8; 1];
        self.read_register(BNO055_CHIP_ID, &mut buf).await?;
        if buf[0] != BNO055_ID {
            defmt::error!("BNO055 chip ID mismatch: expected {:x}, got {:x}", BNO055_ID, buf[0]);
            return Err(Error::ResetFailed);
        } else {
            defmt::info!("BNO055 chip ID: {:x}", buf[0]);
        }
        self.set_mode(BNO055OperationMode::CONFIG_MODE).await?;
        defmt::info!("Set OperationMode to CONFIG_MODE");
        embassy_time::Timer::after_millis(250).await;
        // read operating mode
        self.read_register(BNO055_OPR_MODE, &mut buf).await?;
        if buf[0] != 0xC {
            defmt::error!("BNO055 operation mode mismatch: expected {:x}, got {:x}", 0xC, buf[0]);
            return Err(Error::ResetFailed);
        } else {
            defmt::info!("BNO055 operation mode: {:x}", buf[0]);
        }
        self.write_register(BNO055_PWR_MODE, BNO055PowerMode::NORMAL.bits()).await?;
        embassy_time::Timer::after_millis(250).await;
        // read power mode
        self.read_register(BNO055_PWR_MODE, &mut buf).await?;
        if buf[0] != BNO055PowerMode::NORMAL.bits() {
            defmt::error!("BNO055 power mode mismatch: expected {:x}, got {:x}", BNO055PowerMode::NORMAL.bits(), buf[0]);
            return Err(Error::ResetFailed);
        } else {
            defmt::info!("BNO055 power mode: {:x}", buf[0]);
        }
        defmt::info!("Set power mode to NORMAL");
        embassy_time::Timer::after_millis(250).await;
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        defmt::info!("Write the Page Id to 0x00");
        embassy_time::Timer::after_millis(250).await;
        self.write_register_no_check(BNO055_SYS_TRIGGER, 0x00).await?;
        defmt::info!("Write the SYS_TRIGGER register to 0x00");
        Timer::after_millis(600).await;

        Ok(())
    }

    pub async fn set_mode(&mut self, mode: BNO055OperationMode) -> Result<(), Error> {
        if self.mode != mode {
            self.write_register(BNO055_OPR_MODE, mode.bits()).await?;
            Timer::after_millis(25).await;
            self.mode = mode;
        }
        Ok(())
    }

    async fn reset(&mut self) -> Result<(), Error> {
        self.write_register_no_check(BNO055_SYS_TRIGGER, BNO055_SYS_TRIGGER_RST_SYS_BIT).await?;
        Timer::after_millis(650).await;
        Ok(())
    }

    pub async fn set_power_mode(&mut self, mode: BNO055PowerMode) -> Result<(), Error> {
        self.write_register(BNO055_PWR_MODE, mode.bits()).await?;
        Timer::after_millis(10).await;
        Ok(())
    }

    pub async fn power_mode(&mut self) -> Result<BNO055PowerMode, Error> {
        let mut buf = [0u8; 1];
        self.read_register(BNO055_PWR_MODE, &mut buf).await?;
        Ok(BNO055PowerMode::from_bits_truncate(buf[0]))
    }

        /// Sets current register map page.
    async fn set_page(&mut self, page: BNO055RegisterPage) -> Result<(), Error> {
        self.write_register(BNO055_PAGE_ID, page.bits())
            .await
            .map_err(|_| Error::UartWrite)?;

        Ok(())
    }

    pub async fn get_calibration_status(&mut self) -> Result<BNO055CalibrationStatus, Error> {
        let mut buf = [0u8; 1];
        self.read_register(BNO055_CALIB_STAT, &mut buf).await?;

        let byte = buf[0];
        Ok(BNO055CalibrationStatus {
            sys:   (byte >> 6) & 0x03,
            gyr:  (byte >> 4) & 0x03,
            acc: (byte >> 2) & 0x03,
            mag:   byte & 0x03,
        })
    }

    pub async fn is_fully_calibrated(&mut self) -> Result<bool, Error> {
        let status = self.get_calibration_status().await?;
        Ok(status.sys == 3 && status.gyr == 3 && status.acc == 3 && status.mag == 3)
    }

    /// Reads current calibration profile of the device.
    pub async fn calibration_profile(&mut self) -> Result<BNO055Calibration, Error>
    {
        let prev_mode = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE).await?;
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        let mut buf: [u8; BNO055_CALIB_SIZE] = [0; BNO055_CALIB_SIZE];

        let result = match self.read_register(BNO055_ACC_OFFSET_X_LSB, &mut buf[..]).await {
            Ok(()) => Ok(BNO055Calibration::from_buf(&buf)),
            Err(e) => Err(e)
        };

        self.set_mode(prev_mode).await?;

        result
    }

    /// Sets current calibration profile.
    pub async fn set_calibration_profile(&mut self, calib: BNO055Calibration) -> Result<(), Error>
    {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        let prev_mode = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE).await?;

        let buf_profile = calib.as_bytes();

        // write calibration data to the device as a block
        let result = self.write_block_register(BNO055_ACC_OFFSET_X_LSB, &buf_profile[..]).await;

        self.set_mode(prev_mode).await?;

        result.map_err(|_| Error::UartRead)
    }

    pub async fn get_temperature(&mut self) -> Result<i8, Error> {
        let mut buf = [0u8; 1];
        self.read_register(BNO055_TEMP, &mut buf).await?;
        Ok(buf[0] as i8)
    }

    pub async fn quaternion(&mut self) -> Result<Quaternion, Error> {
        let mut raw = [0u8; 8];

        if let Err(e) = self.read_register(BNO055_QUA_DATA_W_LSB, &mut raw).await {
            defmt::warn!("Failed to read quaternion data: {:?}", e);
            return Err(e);
        }

        let s = i16::from_le_bytes([raw[0], raw[1]]) as f32 / 16384.0;
        let x = i16::from_le_bytes([raw[2], raw[3]]) as f32 / 16384.0;
        let y = i16::from_le_bytes([raw[4], raw[5]]) as f32 / 16384.0;
        let z = i16::from_le_bytes([raw[6], raw[7]]) as f32 / 16384.0;

        Ok(Quaternion { s, v: [x, y, z] })
    }

    async fn read_frame_start(&mut self, expected_start: u8) -> Result<[u8; 2], Error> {
        let mut frame = [0u8; 2];
        loop {
            with_timeout(UART_READ_TIMEOUT, self.uart.read_exact(&mut frame[0..1]))
                .await
                .map_err(|_| Error::Timeout)?
                .map_err(|_| Error::UartRead)?;

            if frame[0] == expected_start {
                with_timeout(UART_READ_TIMEOUT, self.uart.read_exact(&mut frame[1..2]))
                    .await
                    .map_err(|_| Error::Timeout)?
                    .map_err(|_| Error::UartRead)?;
                if frame[1] != expected_start {
                    return Ok(frame);
                }
            }
        }
    }

    pub async fn write_register(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        let payload = [0xAA, 0x00, reg, 1, val];
        let checksum = calc_checksum(&payload[1..]);

        let mut frame = [0u8; 6];
        frame[..5].copy_from_slice(&payload);
        frame[5] = checksum;

        self.uart.write_all(&frame).await.map_err(|_| Error::UartWrite)?;

        let response = self.read_frame_start(0xEE).await?;
        defmt::info!("UART write (fast) response: {:x}, {:x}", response[0], response[1]);
        match response {
            [0xEE, 0x01] => Ok(()),
            [0xEE, code] => Err(Error::AckError(code)),
            _ => Err(Error::UnexpectedWriteResponse),
        }
    }

    pub async fn write_register_no_check(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        let payload = [0xAA, 0x00, reg, 1, val];
        let checksum = calc_checksum(&payload[1..]);

        let mut frame = [0u8; 6];
        frame[..5].copy_from_slice(&payload);
        frame[5] = checksum;

        self.uart.write_all(&frame).await.map_err(|_| Error::UartWrite)?;
        // No need to check the response for this function
        // just wait for the write to complete
        Timer::after_millis(250).await;
        Ok(())
    }

       pub async fn write_register_slow(&mut self, reg: u8, val: u8) -> Result<(), Error> {
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
        let response = self.read_frame_start(0xEE).await?;
        defmt::info!("UART write (slow) response: {:x}, {:x}", response[0], response[1]);
        match response {
            [0xEE, 0x01] => Ok(()),
            [0xEE, code] => Err(Error::AckError(code)),
            _ => Err(Error::UnexpectedWriteResponse),
        }
    }
 
    pub async fn write_block_register(&mut self, reg: u8, values: &[u8]) -> Result<(), Error> {
        let len = values.len();
        if len == 0 || len > 128 {
            return Err(Error::UnexpectedWriteResponse); // or a better error variant
        }

        // Build the frame: header + data
        let mut payload = [0u8; 132]; // 1+1+1+128 + 1 (max size)
        payload[0] = 0xAA;            // Start byte
        payload[1] = 0x00;            // Write command
        payload[2] = reg;             // Register address
        payload[3] = len as u8;       // Number of bytes

        payload[4..4 + len].copy_from_slice(values);
        let checksum = calc_checksum(&payload[1..4 + len]); // skip start byte

        payload[4 + len] = checksum;
        let total_len = 5 + len;

        // Write each byte individually and wait 2ms between each write as the BNO055
        // datasheet recommends a 2ms delay between each byte.
        // This is a workaround for the UART not being able to handle the full frame at once.
        // This is not ideal, but it works for now.
        for byte in &payload[..total_len] {
            let written = self.uart.write(core::slice::from_ref(byte)).await.map_err(|_| Error::UartWrite)?;
            if written != 1 {
                return Err(Error::UartWrite);
            }
            embassy_time::Timer::after_millis(2).await;
        }

        let response = self.read_frame_start(0xEE).await?;
        defmt::info!(
            "UART block write response: 0x{:02X}, 0x{:02X}",
            response[0],
            response[1]
        );

        match response {
            [0xEE, 0x00] => Ok(()),
            [0xEE, code] => Err(Error::AckError(code)),
            _ => Err(Error::UnexpectedWriteResponse),
        }
    }

    pub async fn read_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error> {
        let len = buf.len() as u8;
        let request = [0xAA, 0x01, reg, len];
        
        // Write each byte individually and wait 2ms between each write as the BNO055
        // datasheet recommends a 2ms delay between each byte.
        // This is a workaround for the UART not being able to handle the full frame at once.
        // This is not ideal, but it works for now.
        for byte in &request {
            let written = self.uart.write(core::slice::from_ref(byte)).await.map_err(|_| Error::UartWrite)?;
            if written != 1 {
                return Err(Error::UartWrite);
            }
            embassy_time::Timer::after_millis(2).await;
        }
        let header = self.read_frame_start(0xBB).await?;
        defmt::info!("UART read (fast) response: {:x}, {:x}", header[0], header[1]);
        if header[0] == 0xBB && header[1] as usize != buf.len() {
            return Err(Error::UnexpectedReadLengthResponse);
        }
        if header[0] == 0xEE {
            if header[1] == 0x01 {
                return Ok(());
            }
            return Err(Error::AckError(header[1]));
        }

        self.uart.read_exact(buf).await.map_err(|_| Error::UartRead)?;
        Ok(())
    }
}

fn calc_checksum(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |sum, &val| sum.wrapping_add(val))
}