use bno055::{BNO055OperationMode, Bno055I2c};
use arduino_hal::hal::i2c::I2c;
use arduino_hal::Delay;
use mint::{EulerAngles, Quaternion};

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut delay = arduino_hal::Delay::new();

    let i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        400_000,
    );

    let mut imu = Bno055I2c::new(i2c).with_alternative_address();
    imu.init(&mut delay).unwrap();
    imu.set_mode(BNO055OperationMode::NDOF, &mut delay).unwrap();

    let mut status = imu.get_calibration_status().unwrap();
    // println!("The IMU's calibration status is: {:?}", status);

    // Wait for device to auto-calibrate.
    // Please perform steps necessary for auto-calibration to kick in.
    // Required steps are described in Datasheet section 3.11
    // Page 51, https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf (As of 2021-07-02)
    // println!("- About to begin BNO055 IMU calibration...");
    while !imu.is_fully_calibrated().unwrap() {
        status = imu.get_calibration_status().unwrap();
        delay.delay_ms(1000u16);
        // println!("Calibration status: {:?}", status);
    }

    let calib = imu.calibration_profile(&mut delay).unwrap();
    imu.set_calibration_profile(calib, &mut delay).unwrap();
    // println!("       - Calibration complete!");

    // These are sensor fusion reading using the mint crate that the state will be read into
    let mut euler_angles: EulerAngles<f32, ()>;
    let mut quaternion: Quaternion<f32>;

    loop {
        // Quaternion; due to a bug in the BNO055, this is recommended over Euler Angles
        match imu.quaternion() {
            Ok(val) => {
                quaternion = val;
                // println!("IMU Quaternion: {:?}", quaternion);
                delay.delay_ms(500u16);
            }
            Err(_e) => {
                // eprintln!("{:?}", e);
            }
        }

        // Euler angles, directly read
        match imu.euler_angles() {
            Ok(val) => {
                euler_angles = val;
                // println!("IMU angles: {:?}", euler_angles);
                delay.delay_ms(500u16);
            }
            Err(_e) => {
                // eprintln!("{:?}", e);
            }
        }
    }
}
