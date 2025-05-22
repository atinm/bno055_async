#![allow(clippy::bad_bit_mask)]

//! Bosch Sensortec BNO055 9-axis IMU sensor driver.
//! Datasheet: https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BNO055-DS000.pdf
use embedded_hal_async::i2c::I2c;
use embedded_hal_async::delay::DelayNs;

use byteorder::{ByteOrder, LittleEndian};
pub use mint;

use crate::acc_config;
use crate::regs;
use crate::types::*;
#[cfg(feature = "std")]
mod std;

pub use acc_config::{AccBandwidth, AccConfig, AccGRange, AccOperationMode};
pub use regs::BNO055_ID;

/// All possible errors in this crate
#[derive(Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Error<E> {
    /// I2C bus error
    I2c(E),

    /// Invalid chip ID was read
    InvalidChipId(u8),

    /// Invalid (not applicable) device mode.
    InvalidMode,

    /// Accelerometer configuration error
    AccConfig(acc_config::Error),
}

#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Bno055I2c<I> {
    i2c: I,
    pub mode: BNO055OperationMode,
    use_default_addr: bool,
}

impl<I, E> Bno055I2c<I>
where
    I: I2c<Error = E>,
{
    /// Side-effect-free constructor.
    /// Nothing will be read or written before `init()` call.
    pub fn new(i2c: I) -> Self {
        Bno055I2c {
            i2c,
            mode: BNO055OperationMode::CONFIG_MODE,
            use_default_addr: true,
        }
    }

    /// Destroy driver instance, return I2C bus instance.
    pub fn destroy(self) -> I {
        self.i2c
    }

    /// Enables use of alternative I2C address `regs::BNO055_ALTERNATE_ADDR`.
    pub fn with_alternative_address(mut self) -> Self {
        self.use_default_addr = false;

        self
    }

    /// Initializes the BNO055 device.
    ///
    /// Side-effects:
    /// - Software reset of BNO055
    /// - Sets BNO055 to `CONFIG` mode
    /// - Sets BNO055's power mode to `NORMAL`
    /// - Clears `SYS_TRIGGER` register
    ///
    /// # Usage Example
    ///
    /// ```rust
    /// // use your_chip_hal::{I2c, Delay}; // <- import your chip's I2c and Delay
    /// use bno055::Bno055I2c;
    /// #
    /// # // All of this is needed for example to work:
    /// # use bno055::BNO055_ID;
    /// # use embedded_hal::delay::DelayNs;
    /// # use embedded_hal::i2c::{I2c as I2cTrait, Operation, Error, ErrorType, ErrorKind};
    /// # struct Delay {}
    /// # impl Delay { pub fn new() -> Self { Delay{ } }}
    /// # impl DelayNs for Delay {
    /// #    fn delay_ns(&mut self, ms: u32) {
    /// #        // no-op for example purposes
    /// #    }
    /// # }
    /// # struct I2c {}
    /// # impl I2c { pub fn new() -> Self { I2c { } }}
    /// # #[derive(Debug)]
    /// # struct DummyError {}
    /// # impl Error for DummyError { fn kind(&self) -> ErrorKind { ErrorKind::Other } }
    /// # impl ErrorType for I2c { type Error = DummyError; }
    /// # // 3 calls are made, 2 Writes and 1 Write/Read. We want to mock the 3rd call's read.
    /// # impl I2cTrait for I2c { fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> { match operations.get_mut(1) { Some(Operation::Read(read)) => { read[0] = BNO055_ID; }, _ => {} }; Ok(()) } }
    /// #
    /// # // Actual example:
    /// let mut delay = Delay::new(/* ... */);
    /// let mut i2c = I2c::new(/* ... */);
    /// let mut bno055 = Bno055I2c::new(i2c);
    /// bno055.init(&mut delay)?;
    /// # Result::<(), bno055::Error<DummyError>>::Ok(())
    /// ```
    pub async fn init<D>(&mut self, delay: &mut D) -> Result<(), Error<E>>
    where
        D: DelayNs,
    {
        defmt::info!("Doing Soft reset");
        self.reset(delay).await?;
        defmt::info!("Done Soft reset");

        let id = self.id().await?;
        if id != regs::BNO055_ID {
            return Err(Error::InvalidChipId(id));
        }

        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay).await?;
        self.set_power_mode(BNO055PowerMode::NORMAL).await?;
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        self.write_u8(regs::BNO055_SYS_TRIGGER, 0x00).await
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Resets the BNO055, initializing the register map to default values.
    /// More in section 3.2.
    async fn reset<D>(&mut self, delay: &mut D) -> Result<(), Error<E>>
    where
        D: DelayNs,
    {
        self.write_u8(
            regs::BNO055_SYS_TRIGGER,
            regs::BNO055_SYS_TRIGGER_RST_SYS_BIT,
        )
        .await
        .map_err(Error::I2c)?;

        // As per table 1.2
        delay.delay_ms(650).await;
        Ok(())
    }

    /// Sets the operating mode, see [BNO055OperationMode](enum.BNO055OperationMode.html).
    /// See section 3.3.
    pub async fn set_mode<D>(
        &mut self,
        mode: BNO055OperationMode,
        delay: &mut D,
    ) -> Result<(), Error<E>>
    where
        D: DelayNs,
    {
        if self.mode != mode {
            self.set_page(BNO055RegisterPage::PAGE_0).await?;

            self.mode = mode;

            self.write_u8(regs::BNO055_OPR_MODE, mode.bits())
                .await
                .map_err(Error::I2c)?;

            // Table 3-6 says 19ms to switch to CONFIG_MODE
            delay.delay_ms(19).await;
        }

        Ok(())
    }

    /// Sets the power mode, see [BNO055PowerMode](enum.BNO055PowerMode.html)
    /// See section 3.2
    pub async fn set_power_mode(&mut self, mode: BNO055PowerMode) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        self.write_u8(regs::BNO055_PWR_MODE, mode.bits())
            .await
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Returns BNO055's power mode.
    pub async fn power_mode(&mut self) -> Result<BNO055PowerMode, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        let mode = self.read_u8(regs::BNO055_PWR_MODE).await.map_err(Error::I2c)?;

        Ok(BNO055PowerMode::from_bits_truncate(mode))
    }

    /// Enables/Disables usage of external 32k crystal.
    pub async fn set_external_crystal<D>(
        &mut self,
        ext: bool,
        delay: &mut D,
    ) -> Result<(), Error<E>>
    where
        D: DelayNs,
    {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        let prev = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay).await?;
        self.write_u8(regs::BNO055_SYS_TRIGGER, if ext { 0x80 } else { 0x00 })
            .await
            .map_err(Error::I2c)?;

        self.set_mode(prev, delay).await?;

        Ok(())
    }

    /// Configures axis remap of the device.
    pub async fn set_axis_remap(&mut self, remap: AxisRemap) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        let remap_value = (remap.x().bits() & 0b11)
            | ((remap.y().bits() & 0b11) << 2)
            | ((remap.z().bits() & 0b11) << 4);

        self.write_u8(regs::BNO055_AXIS_MAP_CONFIG, remap_value)
            .await
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Returns axis remap of the device.
    pub async fn axis_remap(&mut self) -> Result<AxisRemap, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        let value = self
            .read_u8(regs::BNO055_AXIS_MAP_CONFIG)
            .await
            .map_err(Error::I2c)?;

        let remap = AxisRemap::new(
            BNO055AxisConfig::from_bits_truncate(value & 0b11),
            BNO055AxisConfig::from_bits_truncate((value >> 2) & 0b11),
            BNO055AxisConfig::from_bits_truncate((value >> 4) & 0b11),
        );

        Ok(remap)
    }

    /// Configures device's axes sign: positive or negative.
    pub async fn set_axis_sign(&mut self, sign: BNO055AxisSign) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        self.write_u8(regs::BNO055_AXIS_MAP_SIGN, sign.bits())
            .await
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Return device's axes sign.
    pub async fn axis_sign(&mut self) -> Result<BNO055AxisSign, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        let value = self
            .read_u8(regs::BNO055_AXIS_MAP_SIGN)
            .await
            .map_err(Error::I2c)?;

        Ok(BNO055AxisSign::from_bits_truncate(value))
    }

    /// Gets the revision of software, bootloader, accelerometer, magnetometer, and gyroscope of
    /// the BNO055 device.
    pub async fn get_revision(&mut self) -> Result<BNO055Revision, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        let mut buf: [u8; 6] = [0; 6];

        self.read_bytes(regs::BNO055_ACC_ID, &mut buf)
            .await
            .map_err(Error::I2c)?;

        Ok(BNO055Revision {
            software: LittleEndian::read_u16(&buf[3..5]),
            bootloader: buf[5],
            accelerometer: buf[0],
            magnetometer: buf[1],
            gyroscope: buf[2],
        })
    }

    /// Returns device's system status.
    pub async fn get_system_status<D>(
        &mut self,
        do_selftest: bool,
        delay: &mut D,
    ) -> Result<BNO055SystemStatus, Error<E>>
    where
        D: DelayNs,
    {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        let selftest = if do_selftest {
            let prev = self.mode;
            self.set_mode(BNO055OperationMode::CONFIG_MODE, delay).await?;

            let sys_trigger = self.read_u8(regs::BNO055_SYS_TRIGGER).await.map_err(Error::I2c)?;

            self.write_u8(regs::BNO055_SYS_TRIGGER, sys_trigger | 0x1)
                .await
                .map_err(Error::I2c)?;

            // Wait for self-test result
            for _ in 0..4 {
                delay.delay_ms(255).await;
            }

            let result = self.read_u8(regs::BNO055_ST_RESULT).await.map_err(Error::I2c)?;

            self.set_mode(prev, delay).await?; // Restore previous mode

            Some(BNO055SelfTestStatus::from_bits_truncate(result))
        } else {
            None
        };

        let status = self.read_u8(regs::BNO055_SYS_STATUS).await.map_err(Error::I2c)?;
        let error = self.read_u8(regs::BNO055_SYS_ERR).await.map_err(Error::I2c)?;

        Ok(BNO055SystemStatus::new(
            BNO055SystemStatusCode::from_bits_truncate(status),
            BNO055SystemErrorCode::from_bits_truncate(error),
            selftest,
        ))
    }

    /// Gets a quaternion (`mint::Quaternion<f32>`) reading from the BNO055.
    /// Available only in sensor fusion modes.
    pub async fn quaternion(&mut self) -> Result<mint::Quaternion<f32>, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        // Device should be in fusion mode to be able to produce quaternions
        if self.mode.is_fusion_enabled() {
            let mut buf: [u8; 8] = [0; 8];
            self.read_bytes(regs::BNO055_QUA_DATA_W_LSB, &mut buf)
                .await
                .map_err(Error::I2c)?;

            let w = LittleEndian::read_i16(&buf[0..2]);
            let x = LittleEndian::read_i16(&buf[2..4]);
            let y = LittleEndian::read_i16(&buf[4..6]);
            let z = LittleEndian::read_i16(&buf[6..8]);

            let scale = 1.0 / ((1 << 14) as f32);

            let x = x as f32 * scale;
            let y = y as f32 * scale;
            let z = z as f32 * scale;
            let w = w as f32 * scale;

            let quat = mint::Quaternion {
                v: mint::Vector3 { x, y, z },
                s: w,
            };

            Ok(quat)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Get Euler angles representation of heading in degrees.
    /// Euler angles is represented as (`roll`, `pitch`, `yaw/heading`).
    /// Available only in sensor fusion modes.
    pub async fn euler_angles(&mut self) -> Result<mint::EulerAngles<f32, ()>, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        // Device should be in fusion mode to be able to produce Euler angles
        if self.mode.is_fusion_enabled() {
            let mut buf: [u8; 6] = [0; 6];

            self.read_bytes(regs::BNO055_EUL_HEADING_LSB, &mut buf)
                .await
                .map_err(Error::I2c)?;

            let heading = LittleEndian::read_i16(&buf[0..2]) as f32;
            let roll = LittleEndian::read_i16(&buf[2..4]) as f32;
            let pitch = LittleEndian::read_i16(&buf[4..6]) as f32;

            let scale = 1f32 / 16f32; // 1 degree = 16 LSB

            let rot = mint::EulerAngles::from([roll * scale, pitch * scale, heading * scale]);

            Ok(rot)
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Get calibration status
    pub async fn get_calibration_status(&mut self) -> Result<BNO055CalibrationStatus, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        let status = self.read_u8(regs::BNO055_CALIB_STAT).await.map_err(Error::I2c)?;

        let sys = (status >> 6) & 0b11;
        let gyr = (status >> 4) & 0b11;
        let acc = (status >> 2) & 0b11;
        let mag = status & 0b11;

        Ok(BNO055CalibrationStatus { sys, gyr, acc, mag })
    }

    /// Checks whether device is fully calibrated or not.
    pub async fn is_fully_calibrated(&mut self) -> Result<bool, Error<E>> {
        let status = self.get_calibration_status().await?;
        Ok(status.mag == 3 && status.gyr == 3 && status.acc == 3 && status.sys == 3)
    }

    /// Reads current calibration profile of the device.
    pub async fn calibration_profile<D>(
        &mut self,
        delay: &mut D,
    ) -> Result<BNO055Calibration, Error<E>>
    where
        D: DelayNs,
    {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        let prev_mode = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay).await?;

        let mut buf: [u8; BNO055_CALIB_SIZE] = [0; BNO055_CALIB_SIZE];

        self.read_bytes(regs::BNO055_ACC_OFFSET_X_LSB, &mut buf[..])
            .await
            .map_err(Error::I2c)?;

        let res = BNO055Calibration::from_buf(&buf);

        self.set_mode(prev_mode, delay).await?;

        Ok(res)
    }

    /// Sets current calibration profile.
    pub async fn set_calibration_profile<D>(
        &mut self,
        calib: BNO055Calibration,
        delay: &mut D,
    ) -> Result<(), Error<E>>
    where
        D: DelayNs,
    {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        let prev_mode = self.mode;
        self.set_mode(BNO055OperationMode::CONFIG_MODE, delay).await?;

        let buf_profile = calib.as_bytes();

        // Combine register address and profile into single buffer
        let buf_reg = [regs::BNO055_ACC_OFFSET_X_LSB; 1];
        let mut buf_with_reg = [0u8; 1 + BNO055_CALIB_SIZE];
        for (to, from) in buf_with_reg
            .iter_mut()
            .zip(buf_reg.iter().chain(buf_profile.iter()))
        {
            *to = *from
        }

        self.i2c
            .write(self.i2c_addr(), &buf_with_reg[..])
            .await
            .map_err(Error::I2c)?;

        self.set_mode(prev_mode, delay).await?;

        Ok(())
    }

    /// Returns device's factory-programmed and constant chip ID.
    /// This ID is device model ID and not a BNO055's unique ID, whic is stored in different register.
    pub async fn id(&mut self) -> Result<u8, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;
        self.read_u8(regs::BNO055_CHIP_ID).await.map_err(Error::I2c)
    }

    /// Returns device's operation mode.
    pub async fn get_mode(&mut self) -> Result<BNO055OperationMode, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        let mode = self.read_u8(regs::BNO055_OPR_MODE).await.map_err(Error::I2c)?;
        let mode = BNO055OperationMode::from_bits_truncate(mode);
        self.mode = mode;

        Ok(mode)
    }

    /// Checks whether the device is in Sensor Fusion mode or not.
    pub fn is_in_fusion_mode(&mut self) -> Result<bool, Error<E>> {
        Ok(self.mode.is_fusion_enabled())
    }

    pub async fn get_acc_config(&mut self) -> Result<AccConfig, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_1).await?;

        let bits = self.read_u8(regs::BNO055_ACC_CONFIG).await.map_err(Error::I2c)?;

        let acc_config = AccConfig::try_from_bits(bits).map_err(Error::AccConfig)?;

        Ok(acc_config)
    }

    pub async fn set_acc_config(&mut self, acc_config: &AccConfig) -> Result<(), Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_1).await?;

        self.write_u8(regs::BNO055_ACC_CONFIG, acc_config.bits())
            .await
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Sets current register map page.
    async fn set_page(&mut self, page: BNO055RegisterPage) -> Result<(), Error<E>> {
        self.write_u8(regs::BNO055_PAGE_ID, page.bits())
            .await
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Reads a vector of sensor data from the device.
    async fn read_vec_raw(&mut self, reg: u8) -> Result<mint::Vector3<i16>, Error<E>> {
        let mut buf: [u8; 6] = [0; 6];

        self.read_bytes(reg, &mut buf).await.map_err(Error::I2c)?;

        let x = LittleEndian::read_i16(&buf[0..2]);
        let y = LittleEndian::read_i16(&buf[2..4]);
        let z = LittleEndian::read_i16(&buf[4..6]);

        Ok(mint::Vector3::from([x, y, z]))
    }

    /// Applies the given scaling to the vector of sensor data from the device.
    fn scale_vec(raw: mint::Vector3<i16>, scaling: f32) -> mint::Vector3<f32> {
        mint::Vector3::from([
            raw.x as f32 * scaling,
            raw.y as f32 * scaling,
            raw.z as f32 * scaling,
        ])
    }

    /// Returns linear acceleration vector in cm/s^2 units.
    /// Available only in sensor fusion modes.
    pub async fn linear_acceleration_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_fusion_enabled() {
            self.set_page(BNO055RegisterPage::PAGE_0).await?;
            self.read_vec_raw(regs::BNO055_LIA_DATA_X_LSB).await
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns linear acceleration vector in m/s^2 units.
    /// Available only in sensor fusion modes.
    pub async fn linear_acceleration(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let linear_acceleration = self.linear_acceleration_fixed().await?;
        let scaling = 1f32 / 100f32; // 1 m/s^2 = 100 lsb
        Ok(Self::scale_vec(linear_acceleration, scaling))
    }

    /// Returns gravity vector in cm/s^2 units.
    /// Available only in sensor fusion modes.
    pub async fn gravity_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_fusion_enabled() {
            self.set_page(BNO055RegisterPage::PAGE_0).await?;
            self.read_vec_raw(regs::BNO055_GRV_DATA_X_LSB).await
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns gravity vector in m/s^2 units.
    /// Available only in sensor fusion modes.
    pub async fn gravity(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let gravity = self.gravity_fixed().await?;
        let scaling = 1f32 / 100f32; // 1 m/s^2 = 100 lsb
        Ok(Self::scale_vec(gravity, scaling))
    }

    /// Returns current accelerometer data in cm/s^2 units.
    /// Available only in modes in which accelerometer is enabled.
    pub async fn accel_data_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_accel_enabled() {
            self.set_page(BNO055RegisterPage::PAGE_0).await?;
            self.read_vec_raw(regs::BNO055_ACC_DATA_X_LSB).await
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns current accelerometer data in m/s^2 units.
    /// Available only in modes in which accelerometer is enabled.
    pub async fn accel_data(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let a = self.accel_data_fixed().await?;
        let scaling = 1f32 / 100f32; // 1 m/s^2 = 100 lsb
        Ok(Self::scale_vec(a, scaling))
    }

    /// Returns current gyroscope data in 1/16th deg/s units.
    /// Available only in modes in which gyroscope is enabled.
    pub async fn gyro_data_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_gyro_enabled() {
            self.set_page(BNO055RegisterPage::PAGE_0).await?;
            self.read_vec_raw(regs::BNO055_GYR_DATA_X_LSB).await
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns current gyroscope data in deg/s units.
    /// Available only in modes in which gyroscope is enabled.
    pub async fn gyro_data(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let g = self.gyro_data_fixed().await?;
        let scaling = 1f32 / 16f32; // 1 deg/s = 16 lsb
        Ok(Self::scale_vec(g, scaling))
    }

    /// Returns current magnetometer data in 1/16th uT units.
    /// Available only in modes in which magnetometer is enabled.
    pub async fn mag_data_fixed(&mut self) -> Result<mint::Vector3<i16>, Error<E>> {
        if self.mode.is_mag_enabled() {
            self.set_page(BNO055RegisterPage::PAGE_0).await?;
            self.read_vec_raw(regs::BNO055_MAG_DATA_X_LSB).await
        } else {
            Err(Error::InvalidMode)
        }
    }

    /// Returns current magnetometer data in uT units.
    /// Available only in modes in which magnetometer is enabled.
    pub async fn mag_data(&mut self) -> Result<mint::Vector3<f32>, Error<E>> {
        let m = self.mag_data_fixed().await?;
        let scaling = 1f32 / 16f32; // 1 uT = 16 lsb
        Ok(Self::scale_vec(m, scaling))
    }

    /// Returns current temperature of the chip (in degrees Celsius).
    pub async fn temperature(&mut self) -> Result<i8, Error<E>> {
        self.set_page(BNO055RegisterPage::PAGE_0).await?;

        // Read temperature signed byte
        let temp = self.read_u8(regs::BNO055_TEMP).await.map_err(Error::I2c)? as i8;
        Ok(temp)
    }

    #[inline(always)]
    fn i2c_addr(&self) -> u8 {
        if !self.use_default_addr {
            regs::BNO055_ALTERNATE_ADDR
        } else {
            regs::BNO055_DEFAULT_ADDR
        }
    }

    async fn read_u8(&mut self, reg: u8) -> Result<u8, E> {
        let mut byte: [u8; 1] = [0; 1];
        self.i2c.write_read(self.i2c_addr(), &[reg], &mut byte).await?;
        Ok(byte[0])
    }

    async fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(self.i2c_addr(), &[reg], buf).await
    }

    async fn write_u8(&mut self, reg: u8, value: u8) -> Result<(), E> {
        self.i2c.write(self.i2c_addr(), &[reg, value]).await?;
        Ok(())
    }
}
