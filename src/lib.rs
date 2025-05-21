#![doc(html_root_url = "https://docs.rs/bno055/0.5.0")]
#![cfg_attr(not(feature = "std"), no_std)]

pub mod i2c;
pub mod uart;
pub mod regs;
pub mod types;
pub mod acc_config;
