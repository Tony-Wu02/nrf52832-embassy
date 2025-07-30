#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize
    embassy_nrf::init(Default::default());
    
    // Simple info test
    info!("Hello from nRF52832!");
    
    // Wait and repeat
    loop {
        info!("Tick...");
        Timer::after_millis(2000).await;
    }
} 