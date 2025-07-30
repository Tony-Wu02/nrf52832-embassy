#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize the nRF52832 first
    let p = embassy_nrf::init(Default::default());
    
    // Wait a bit for everything to stabilize
    Timer::after_millis(100).await;
    
    // Now try to use info! after initialization
    info!("Starting debug test program...");
    
    // Configure LED
    let mut led = Output::new(p.P0_17, Level::Low, OutputDrive::Standard);
    
    info!("LED configured on P0_17");
    
    let mut counter = 0;
    
    loop {
        counter += 1;
        
        // Turn LED on
        led.set_high();
        info!("LED ON - Counter: {}", counter);
        Timer::after_millis(1000).await;
        
        // Turn LED off
        led.set_low();
        info!("LED OFF - Counter: {}", counter);
        Timer::after_millis(1000).await;
        
        // Add a longer delay every 5 cycles
        if counter % 5 == 0 {
            info!("Pause for 2 seconds...");
            Timer::after_millis(2000).await;
        }
    }
} 