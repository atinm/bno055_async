use bno055::{BNO055OperationMode, Bno055Uart};
use arduino_hal::Delay;
use mint::{EulerAngles, Quaternion};

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut delay = Delay::new();

    // Setup UART on pins D0 (RX) and D1 (TX)
    let serial = arduino_hal::usart::Usart::new(
        dp.USART0,
        pins.d0,
        pins.d1.into_output(),
        115200,
    );

    let mut imu = Bno055Uart::new(serial);
    imu.init(&mut delay).unwrap();
    imu.set_mode(BNO055OperationMode::NDOF, &mut delay).unwrap();

    let mut status = imu.get_calibration_status().unwrap();
    while !imu.is_fully_calibrated().unwrap() {
        status = imu.get_calibration_status().unwrap();
        delay.delay_ms(1000u16);
    }

    let calib = imu.calibration_profile(&mut delay).unwrap();
    imu.set_calibration_profile(calib, &mut delay).unwrap();

    let mut euler_angles: EulerAngles<f32, ()>;
    let mut quaternion: Quaternion<f32>;

    loop {
        match imu.quaternion() {
            Ok(val) => {
                quaternion = val;
                delay.delay_ms(500u16);
            }
            Err(_) => {}
        }

        match imu.euler_angles() {
            Ok(val) => {
                euler_angles = val;
                delay.delay_ms(500u16);
            }
            Err(_) => {}
        }
    }
}