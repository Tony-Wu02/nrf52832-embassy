#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize hardware
    let p = embassy_nrf::init(Default::default());
    let mut led = Output::new(p.P0_17, Level::Low, OutputDrive::Standard);
    
    // Wait for system to stabilize
    Timer::after_millis(1000).await;
    
    info!("=== RTT Test Program Started ===");
    info!("This message should be visible in RTT viewer");
    info!("LED will blink every 2 seconds");
    
    let mut counter = 0;
    
    loop {
        counter += 1;
        
        led.set_high();
        info!("LED ON - Counter: {}", counter);
        Timer::after_millis(2000).await;
        
        led.set_low();
        info!("LED OFF - Counter: {}", counter);
        Timer::after_millis(2000).await;
        
        if counter % 5 == 0 {
            info!("=== Summary: {} cycles completed ===", counter);
        }
    }
} 