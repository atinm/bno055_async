#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;
use defmt::{info, warn};

use embassy_rp::bind_interrupts;
use embassy_rp::i2c::I2c;
use embassy_rp::peripherals::{I2C0};
use embassy_executor::Spawner;
use embassy_time::{Delay, Timer};
use bno055::types::BNO055OperationMode;
use bno055::i2c::Bno055I2c;


bind_interrupts!(struct Irqs {
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<I2C0>;
});

#[no_mangle]
extern "C" fn _defmt_timestamp() -> u64 {
    use embassy_time::Instant;
    Instant::now().as_ticks()
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_0;
    let scl = p.PIN_1;
    let i2c = I2c::new_async(p.I2C0, scl, sda, Irqs, embassy_rp::i2c::Config::default());

    let mut delay = Delay;

    let mut bno055 = Bno055I2c::new(i2c);
    bno055.init(&mut delay).await.unwrap();
    bno055.set_mode(BNO055OperationMode::NDOF, &mut delay).await.unwrap();

    info!("Waiting for BNO055 to calibrate");
    while !bno055.is_fully_calibrated().await.unwrap() {}

    info!("BNO055 calibrated");

    loop {
        match bno055.quaternion().await {
            Ok(quat) => info!(
                "Quaternion: w={} x={} y={} z={}",
                quat.s, quat.v.x, quat.v.y, quat.v.z
            ),
            Err(e) => warn!("BNO055 quaternion read error: {:?}", e),
        }

        Timer::after_millis(500).await;
    }
}
