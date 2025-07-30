#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize hardware first
    let p = embassy_nrf::init(Default::default());
    
    // Configure LED for visual feedback
    let mut led = Output::new(p.P0_17, Level::Low, OutputDrive::Standard);
    
    // Wait for system to stabilize
    Timer::after_millis(500).await;
    
    // Now it's safe to use info!
    info!("System initialized successfully");
    
    let mut counter = 0;
    
    loop {
        counter += 1;
        
        // Visual feedback with LED
        led.set_high();
        info!("LED ON - Counter: {}", counter);
        Timer::after_millis(1000).await;
        
        led.set_low();
        info!("LED OFF - Counter: {}", counter);
        Timer::after_millis(1000).await;
        
        // Every 5 cycles, add extra info
        if counter % 5 == 0 {
            info!("Completed {} cycles", counter);
            Timer::after_millis(500).await;
        }
    }
} 