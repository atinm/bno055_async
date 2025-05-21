#[cfg(not(feature = "defmt-03"))]
use bitflags::bitflags;
#[cfg(feature = "defmt-03")]
use defmt::bitflags;

pub use mint;
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

bitflags! {
    #[cfg_attr(not(feature = "defmt-03"), derive(Debug, Clone, Copy, PartialEq, Eq))]
    pub struct BNO055AxisConfig: u8 {
        const AXIS_AS_X = 0b00;
        const AXIS_AS_Y = 0b01;
        const AXIS_AS_Z = 0b10;
    }
}

#[allow(clippy::misnamed_getters)]
impl AxisRemap {
    pub fn x(&self) -> BNO055AxisConfig {
        self.x
    }

    pub fn y(&self) -> BNO055AxisConfig {
        self.x
    }

    pub fn z(&self) -> BNO055AxisConfig {
        self.z
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AxisRemap {
    x: BNO055AxisConfig,
    y: BNO055AxisConfig,
    z: BNO055AxisConfig,
}

#[derive(Debug)]
pub struct AxisRemapBuilder {
    remap: AxisRemap,
}

impl AxisRemap {
    pub fn new(x: BNO055AxisConfig, y: BNO055AxisConfig, z: BNO055AxisConfig) -> Self {
        Self { x, y, z }
    }

    pub fn builder() -> AxisRemapBuilder {
        AxisRemapBuilder {
            remap: AxisRemap {
                x: BNO055AxisConfig::AXIS_AS_X,
                y: BNO055AxisConfig::AXIS_AS_Y,
                z: BNO055AxisConfig::AXIS_AS_Z,
            },
        }
    }
}

impl AxisRemapBuilder {
    pub fn swap_x_with(mut self, to: BNO055AxisConfig) -> AxisRemapBuilder {
        let old_x = self.remap.x;

        match to {
            BNO055AxisConfig::AXIS_AS_X => self.remap.x = old_x,
            BNO055AxisConfig::AXIS_AS_Y => self.remap.y = old_x,
            BNO055AxisConfig::AXIS_AS_Z => self.remap.z = old_x,

            _ => (),
        }

        self.remap.x = to;

        AxisRemapBuilder { remap: self.remap }
    }

    pub fn swap_y_with(mut self, to: BNO055AxisConfig) -> AxisRemapBuilder {
        let old_y = self.remap.y;

        match to {
            BNO055AxisConfig::AXIS_AS_X => self.remap.x = old_y,
            BNO055AxisConfig::AXIS_AS_Y => self.remap.y = old_y,
            BNO055AxisConfig::AXIS_AS_Z => self.remap.z = old_y,

            _ => (),
        }

        self.remap.y = to;

        AxisRemapBuilder { remap: self.remap }
    }

    pub fn swap_z_with(mut self, to: BNO055AxisConfig) -> AxisRemapBuilder {
        let old_z = self.remap.z;

        match to {
            BNO055AxisConfig::AXIS_AS_X => self.remap.x = old_z,
            BNO055AxisConfig::AXIS_AS_Y => self.remap.y = old_z,
            BNO055AxisConfig::AXIS_AS_Z => self.remap.z = old_z,

            _ => (),
        }

        self.remap.z = to;

        AxisRemapBuilder { remap: self.remap }
    }

    fn is_invalid(&self) -> bool {
        // Each axis must be swapped only once,
        // For example, one cannot remap X to Y and Z to Y at the same time, or similar.
        // See datasheet, section 3.4.
        self.remap.x == self.remap.y || self.remap.y == self.remap.z || self.remap.z == self.remap.x
    }

    #[allow(clippy::result_unit_err)]
    pub fn build(self) -> Result<AxisRemap, ()> {
        if self.is_invalid() {
            Err(())
        } else {
            Ok(self.remap)
        }
    }
}

bitflags! {
    #[cfg_attr(not(feature = "defmt-03"), derive(Debug, Clone, Copy, PartialEq, Eq))]
    pub struct BNO055AxisSign: u8 {
        const X_NEGATIVE = 0b100;
        const Y_NEGATIVE = 0b010;
        const Z_NEGATIVE = 0b001;
    }
}

bitflags! {
    #[cfg_attr(not(feature = "defmt-03"), derive(Debug, Clone, Copy, PartialEq, Eq))]
    pub struct BNO055SystemStatusCode: u8 {
        const SYSTEM_IDLE = 0;
        const SYSTEM_ERROR = 1;
        const INIT_PERIPHERALS = 2;
        const SYSTEM_INIT = 3;
        const EXECUTING = 4;
        const RUNNING = 5;
        const RUNNING_WITHOUT_FUSION = 6;
    }
}

bitflags! {
    /// Possible BNO055 errors.
    #[cfg_attr(not(feature = "defmt-03"), derive(Debug, Clone, Copy, PartialEq, Eq))]
    pub struct BNO055SystemErrorCode: u8 {
        const NONE = 0;
        const PERIPHERAL_INIT = 1;
        const SYSTEM_INIT = 2;
        const SELF_TEST = 3;
        const REGISTER_MAP_VALUE = 4;
        const REGISTER_MAP_ADDRESS = 5;
        const REGISTER_MAP_WRITE = 6;
        const LOW_POWER_MODE_NOT_AVAIL = 7;
        const ACCEL_POWER_MODE_NOT_AVAIL = 8;
        const FUSION_ALGO_CONFIG = 9;
        const SENSOR_CONFIG = 10;
    }
}

bitflags! {
    /// BNO055 self-test status bit flags.
    #[cfg_attr(not(feature = "defmt-03"), derive(Debug, Clone, Copy, PartialEq, Eq))]
    pub struct BNO055SelfTestStatus: u8 {
        const ACC_OK = 0b0001;
        const MAG_OK = 0b0010;
        const GYR_OK = 0b0100;
        const SYS_OK = 0b1000;
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055SystemStatus {
    status: BNO055SystemStatusCode,
    selftest: Option<BNO055SelfTestStatus>,
    error: BNO055SystemErrorCode,
}

impl BNO055SystemStatus {
    pub fn new(
        status: BNO055SystemStatusCode,
        error: BNO055SystemErrorCode,
        selftest: Option<BNO055SelfTestStatus>,
    ) -> Self {
        Self { status, error, selftest }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055Revision {
    pub software: u16,
    pub bootloader: u8,
    pub accelerometer: u8,
    pub magnetometer: u8,
    pub gyroscope: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(C)]
pub struct BNO055Calibration {
    pub acc_offset_x_lsb: u8,
    pub acc_offset_x_msb: u8,
    pub acc_offset_y_lsb: u8,
    pub acc_offset_y_msb: u8,
    pub acc_offset_z_lsb: u8,
    pub acc_offset_z_msb: u8,

    pub mag_offset_x_lsb: u8,
    pub mag_offset_x_msb: u8,
    pub mag_offset_y_lsb: u8,
    pub mag_offset_y_msb: u8,
    pub mag_offset_z_lsb: u8,
    pub mag_offset_z_msb: u8,

    pub gyr_offset_x_lsb: u8,
    pub gyr_offset_x_msb: u8,
    pub gyr_offset_y_lsb: u8,
    pub gyr_offset_y_msb: u8,
    pub gyr_offset_z_lsb: u8,
    pub gyr_offset_z_msb: u8,

    pub acc_radius_lsb: u8,
    pub acc_radius_msb: u8,
    pub mag_radius_lsb: u8,
    pub mag_radius_msb: u8,
}

/// BNO055's calibration profile size.
pub const BNO055_CALIB_SIZE: usize = core::mem::size_of::<BNO055Calibration>();

impl BNO055Calibration {
    pub fn from_buf(buf: &[u8; BNO055_CALIB_SIZE]) -> BNO055Calibration {
        unsafe { core::ptr::read(buf.as_ptr() as *const _) }
    }

    pub fn as_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                (self as *const _) as *const u8,
                ::core::mem::size_of::<BNO055Calibration>(),
            )
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct BNO055CalibrationStatus {
    pub sys: u8,
    pub gyr: u8,
    pub acc: u8,
    pub mag: u8,
}

bitflags! {
    /// Possible BNO055 register map pages.
    #[cfg_attr(not(feature = "defmt-03"), derive(Debug, Clone, Copy, PartialEq, Eq))]
    pub struct BNO055RegisterPage: u8 {
        const PAGE_0 = 0;
        const PAGE_1 = 1;
    }
}

bitflags! {
    /// Possible BNO055 power modes.
    #[cfg_attr(not(feature = "defmt-03"), derive(Debug, Clone, Copy, PartialEq, Eq))]
    pub struct BNO055PowerMode: u8 {
        const NORMAL = 0b00;
        const LOW_POWER = 0b01;
        const SUSPEND = 0b10;
    }
}

bitflags! {
    /// Possible BNO055 operation modes.
    #[cfg_attr(not(feature = "defmt-03"), derive(Debug, Clone, Copy, PartialEq, Eq))]
    pub struct BNO055OperationMode: u8 {
        const CONFIG_MODE = 0b0000;
        const ACC_ONLY = 0b0001;
        const MAG_ONLY = 0b0010;
        const GYRO_ONLY = 0b0011;
        const ACC_MAG = 0b0100;
        const ACC_GYRO = 0b0101;
        const MAG_GYRO = 0b0110;
        const AMG = 0b0111;
        const IMU = 0b1000;
        const COMPASS = 0b1001;
        const M4G = 0b1010;
        const NDOF_FMC_OFF = 0b1011;
        const NDOF = 0b1100;
    }
}

impl BNO055OperationMode {
    pub fn is_fusion_enabled(&self) -> bool {
        matches!(
            *self,
            Self::IMU | Self::COMPASS | Self::M4G | Self::NDOF_FMC_OFF | Self::NDOF,
        )
    }

    pub fn is_accel_enabled(&self) -> bool {
        matches!(
            *self,
            Self::ACC_ONLY
                | Self::ACC_MAG
                | Self::ACC_GYRO
                | Self::AMG
                | Self::IMU
                | Self::COMPASS
                | Self::M4G
                | Self::NDOF_FMC_OFF
                | Self::NDOF,
        )
    }

    pub fn is_gyro_enabled(&self) -> bool {
        matches!(
            *self,
            Self::GYRO_ONLY
                | Self::ACC_GYRO
                | Self::MAG_GYRO
                | Self::AMG
                | Self::IMU
                | Self::NDOF_FMC_OFF
                | Self::NDOF,
        )
    }

    pub fn is_mag_enabled(&self) -> bool {
        matches!(
            *self,
            Self::MAG_ONLY
                | Self::ACC_MAG
                | Self::MAG_GYRO
                | Self::AMG
                | Self::COMPASS
                | Self::M4G
                | Self::NDOF_FMC_OFF
                | Self::NDOF,
        )
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    pub s: f32,
    pub v: [f32; 3],
}
