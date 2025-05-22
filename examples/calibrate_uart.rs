#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::main;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{UART0, PIN_0, PIN_1};
use embassy_rp::uart::{BufferedUart, BufferedInterruptHandler, Config};
use embassy_time::{Delay, Timer};
use bno055::types::BNO055OperationMode;
use bno055::uart::Bno055Uart;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
});

static TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = embassy_rp::init(Default::default());

    let uart = BufferedUart::new(
        p.UART0,
        Irqs,
        p.PIN_0,
        p.PIN_1,
        Config::default(),
        RX_BUF.init([0; 256]),
        TX_BUF.init([0; 256]),
    );

    let mut imu = Bno055Uart::new(uart);
    let mut delay = Delay;

    imu.init(&mut delay).await.unwrap();
    imu.set_mode(BNO055OperationMode::NDOF, &mut delay).await.unwrap();

    while !imu.is_fully_calibrated().await.unwrap() {
        Timer::after_millis(1000).await;
    }

    let calib = imu.calibration_profile(&mut delay).await.unwrap();
    imu.set_calibration_profile(calib, &mut delay).await.unwrap();

    loop {
        if let Ok(q) = imu.quaternion().await {
            defmt::info!("Quaternion: {:?}", q);
        }

        Timer::after_millis(1000).await;
    }
}