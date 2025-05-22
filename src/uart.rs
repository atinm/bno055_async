// uart.rs

use embedded_io_async::{Read, Write};
use embassy_time::{Duration, with_timeout};
use crate::regs::*;
use crate::types::*;
use embedded_hal_async::delay::DelayNs;

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

    pub async fn init(&mut self, delay: &mut impl DelayNs) -> Result<(), Error> {
        defmt::info!("Doing Soft reset");
        self.reset(delay).await?;
        delay.delay_ms(650).await;
        defmt::info!("Done Soft reset");
        let mut buf = [0u8; 1];
        // read operating mode
        self.read_register(BNO055_OPR_MODE, &mut buf).await?;
        if buf[0] != 0x10 {
            defmt::error!("BNO055 operation mode mismatch: expected 0b{:b}, got 0b{:b}", 0x0, buf[0]);
            return Err(Error::ResetFailed);
        } else {
            defmt::info!("BNO055 operation mode: {:x}", buf[0]);
        }

        // read the chip ID
        let mut success = false;
        for attempt in 1..=10 {
            match self.read_register(BNO055_CHIP_ID, &mut buf).await {
                Ok(_) if buf[0] == BNO055_ID => {
                    defmt::info!("BNO055 chip ID confirmed: {:x}", buf[0]);
                    success = true;
                    break;
                }
                Ok(_) => {
                    defmt::warn!(
                        "Attempt {}: BNO055 chip ID mismatch: expected {:x}, got {:x}",
                        attempt,
                        BNO055_ID,
                        buf[0]
                    );
                }
                Err(e) => {
                    defmt::warn!(
                        "Attempt {}: Error reading BNO055 chip ID: {:?}",
                        attempt,
                        e
                    );
                }
            }

            delay.delay_ms(100).await;
        }

        if !success {
            defmt::error!("Failed to confirm BNO055 chip ID after multiple attempts.");
            //return Err(Error::ResetFailed);
        }
        defmt::info!("Setting OperationMode to CONFIG_MODE");
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay).await?;
        defmt::info!("Set OperationMode to CONFIG_MODE");
        delay.delay_ms(250).await;
        // read operating mode
        self.read_register(BNO055_OPR_MODE, &mut buf).await?;
        if buf[0] != 0x10 {
            defmt::error!("BNO055 operation mode mismatch: expected {:x}, got {:x}", 0x10, buf[0]);
            return Err(Error::ResetFailed);
        } else {
            defmt::info!("BNO055 operation mode: {:x}", buf[0]);
        }
        defmt::info!("Setting PowerMode to NORMAL");
        self.set_power_mode(BNO055PowerMode::NORMAL, delay).await?;
        defmt::info!("Set power mode to NORMAL");
        defmt::info!("Write the Page Id to 0x00");
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        self.write_register_no_check(BNO055_SYS_TRIGGER, 0x00).await?;
        defmt::info!("Write the SYS_TRIGGER register to 0x00");
        delay.delay_ms(650).await;

        Ok(())
    }

    pub async fn set_mode(&mut self, mode: BNO055OperationMode, delay: &mut impl DelayNs) -> Result<(), Error> {
        if self.mode != mode {
            self.write_register(BNO055_OPR_MODE, mode.bits()).await?;
            delay.delay_ms(25).await;
            self.mode = mode;
        }
        Ok(())
    }

    pub async fn mode(&mut self, delay: &mut impl DelayNs) -> Result<BNO055OperationMode, Error> {
        let mut buf = [0u8; 1];
        self.read_register(BNO055_OPR_MODE, &mut buf).await?;
        Ok(BNO055OperationMode::from_bits_truncate(buf[0]))
    }

    async fn reset(&mut self, delay: &mut impl DelayNs) -> Result<(), Error> {
        self.write_register_no_check(BNO055_SYS_TRIGGER, BNO055_SYS_TRIGGER_RST_SYS_BIT).await?;
        let mut frame = [0u8; 1];

        with_timeout(UART_READ_TIMEOUT, self.uart.read_exact(&mut frame))
            .await
            .map_err(|_| Error::Timeout)?
            .map_err(|_| Error::UartRead)?;

        if frame[0] != 0xEE {
            defmt::warn!("Unexpected response byte on reset: {:x}, expected: {:x}", frame[0], 0xEE);
        }
        delay.delay_ms(650).await;
        Ok(())
    }

    pub async fn set_power_mode(&mut self, mode: BNO055PowerMode, delay: &mut impl DelayNs) -> Result<(), Error> {
        self.write_register_slow(BNO055_PWR_MODE, mode.bits()).await?;
        delay.delay_ms(10).await;
        Ok(())
    }

    pub async fn power_mode(&mut self) -> Result<BNO055PowerMode, Error> {
        let mut buf = [0u8; 1];
        self.read_register(BNO055_PWR_MODE, &mut buf).await?;
        Ok(BNO055PowerMode::from_bits_truncate(buf[0]))
    }

        /// Sets current register map page.
    async fn set_page(&mut self, page: BNO055RegisterPage) -> Result<(), Error> {
        self.write_register_no_check(BNO055_PAGE_ID, page.bits())
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
    pub async fn calibration_profile(&mut self, delay: &mut impl DelayNs) -> Result<BNO055Calibration, Error> {
        let prev_mode = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay).await?;

        let result = match async {
            self.set_page(BNO055RegisterPage::PAGE_0).await?;
            let mut acc_offsets: [u8; 6] = [0; 6];
            self.read_register(BNO055_ACC_OFFSET_X_LSB, &mut acc_offsets[..]).await?;
            let mut mag_offsets: [u8; 6] = [0; 6];
            self.read_register(BNO055_MAG_OFFSET_X_LSB, &mut mag_offsets[..]).await?;
            let mut gyr_offsets: [u8; 6] = [0; 6];
            self.read_register(BNO055_GYR_OFFSET_X_LSB, &mut gyr_offsets[..]).await?;
            let mut acc_radius: [u8; 2] = [0; 2];
            self.read_register(BNO055_ACC_RADIUS_LSB, &mut acc_radius[..]).await?;
            let mut mag_radius: [u8; 2] = [0; 2];
            self.read_register(BNO055_MAG_RADIUS_LSB, &mut mag_radius[..]).await?;
            // merge all the data into a single buffer
            let mut buf = [0u8; 22];
            buf[0..6].copy_from_slice(&acc_offsets[..]);
            buf[6..12].copy_from_slice(&mag_offsets[..]);
            buf[12..18].copy_from_slice(&gyr_offsets[..]);
            buf[18..20].copy_from_slice(&acc_radius[..]);
            buf[20..22].copy_from_slice(&mag_radius[..]);
            Ok(BNO055Calibration::from_buf(&buf))
        }.await {
            Ok(val) => {
                self.set_mode(prev_mode, delay).await?;
                Ok(val)
            }
            Err(e) => {
                self.set_mode(prev_mode, delay).await?;
                Err(e)
            }
        };
        result
    }

    /// Sets current calibration profile.
    pub async fn set_calibration_profile(&mut self, calib: BNO055Calibration, delay: &mut impl DelayNs) -> Result<(), Error> {
        let prev_mode = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay).await?;

        let result = match async {
            let buf_profile = calib.as_bytes();
            self.write_block_register(BNO055_ACC_OFFSET_X_LSB, &buf_profile[..]).await.map_err(|_| Error::UartRead)
        }.await {
            Ok(_) => {
                self.set_mode(prev_mode, delay).await?;
                Ok(())
            }
            Err(e) => {
                self.set_mode(prev_mode, delay).await?;
                Err(e)
            }
        };
        result
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

    /// Flush any pending/stale bytes from the UART receive buffer.
    async fn flush_uart_rx(&mut self) {
        let mut byte = [0u8; 1];
        loop {
            match with_timeout(UART_READ_TIMEOUT, self.uart.read_exact(&mut byte)).await {
                Ok(Ok(())) => {
                    defmt::warn!("Flushed stray byte: 0x{:02X}", byte[0]);
                    continue;
                }
                _ => break, // either timeout or error => buffer is empty
            }
        }
    }    

    async fn read_frame_start(&mut self) -> Result<[u8; 2], Error> {
        let mut frame = [0u8; 2];

        loop {
            // Read the first byte with timeout
            with_timeout(UART_READ_TIMEOUT, self.uart.read_exact(&mut frame[0..1]))
                .await
                .map_err(|_| Error::Timeout)?
                .map_err(|_| Error::UartRead)?;

            match frame[0] {
                0xEE | 0xBB => {
                    // Read the second byte
                    with_timeout(UART_READ_TIMEOUT, self.uart.read_exact(&mut frame[1..2]))
                        .await
                        .map_err(|_| Error::Timeout)?
                        .map_err(|_| Error::UartRead)?;
                    return Ok(frame);
                }
                other => {
                    defmt::warn!("Unexpected start byte: {:x}, expected 0xEE or 0xBB", other);
                    // Flush the UART RX buffer to discard any stray bytes
                    self.flush_uart_rx().await;
                    continue;
                }
            }
        }
    }

    pub async fn write_register(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        self.flush_uart_rx().await;

        let payload = [0xAA, 0x00, reg, 1, val];
        let checksum = calc_checksum(&payload[1..]);

        let mut frame = [0u8; 6];
        frame[..5].copy_from_slice(&payload);
        frame[5] = checksum;

        //self.uart.write_all(&frame).await.map_err(|_| Error::UartWrite)?;
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

        let response = self.read_frame_start().await?;
        defmt::info!("UART write (fast) response: {:x}, {:x}", response[0], response[1]);
        match response {
            [0xEE, 0x01] => Ok(()),
            [0xEE, code] => Err(Error::AckError(code)),
            _ => Err(Error::UnexpectedWriteResponse),
        }
    }

    pub async fn write_register_no_check(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        self.flush_uart_rx().await;

        let payload = [0xAA, 0x00, reg, 1, val];
        let checksum = calc_checksum(&payload[1..]);

        let mut frame = [0u8; 6];
        frame[..5].copy_from_slice(&payload);
        frame[5] = checksum;

        //self.uart.write_all(&frame).await.map_err(|_| Error::UartWrite)?;
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

        // No need to check the response for this function
        // just wait for the write to complete
        // Use a fixed delay here
        embassy_time::Timer::after_millis(250).await;
        Ok(())
    }

       pub async fn write_register_slow(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        self.flush_uart_rx().await;

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
        let response = self.read_frame_start().await?;
        defmt::info!("UART write (slow) response: {:x}, {:x}", response[0], response[1]);
        match response {
            [0xEE, 0x01] => Ok(()),
            [0xEE, code] => Err(Error::AckError(code)),
            _ => Err(Error::UnexpectedWriteResponse),
        }
    }
 
    pub async fn write_block_register(&mut self, reg: u8, values: &[u8]) -> Result<(), Error> {
        self.flush_uart_rx().await;

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

        let response = self.read_frame_start().await?;
        defmt::info!(
            "UART block write response: 0x{:02X}, 0x{:02X}",
            response[0],
            response[1]
        );

        match response {
            [0xEE, 0x01] => Ok(()),
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
                defmt::error!("UART read (fast) response, wrote: {}", written);
                return Err(Error::UartWrite);
            }
            embassy_time::Timer::after_millis(2).await;
        }
        let header = match self.read_frame_start().await {
            Ok(h) => h,
            Err(e) => {
                defmt::warn!("Error in read_frame_start: {:?}", e);
                return Err(e);
            }
        };
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
