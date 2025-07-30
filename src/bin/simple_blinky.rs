#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize the nRF52832
    let p = embassy_nrf::init(Default::default());
    
    // Configure LED1 (P0_17) as output
    let mut led = Output::new(p.P0_17, Level::Low, OutputDrive::Standard);

    loop {
        // Turn LED on
        led.set_high();
        Timer::after_millis(1000).await;
        
        // Turn LED off
        led.set_low();
        Timer::after_millis(1000).await;
    }
} 