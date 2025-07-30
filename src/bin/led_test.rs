#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    
    // Try multiple common LED pins
    let mut led1 = Output::new(p.P0_13, Level::Low, OutputDrive::Standard);
    let mut led2 = Output::new(p.P0_14, Level::Low, OutputDrive::Standard);
    let mut led3 = Output::new(p.P0_15, Level::Low, OutputDrive::Standard);
    let mut led4 = Output::new(p.P0_16, Level::Low, OutputDrive::Standard);
    let mut led5 = Output::new(p.P0_17, Level::Low, OutputDrive::Standard);

    info!("Starting LED test program...");
    info!("Testing multiple LED pins...");

    loop {
        // Test P0_13
        info!("Testing P0_13 - LED should blink");
        led1.set_high();
        Timer::after_millis(500).await;
        led1.set_low();
        Timer::after_millis(500).await;
        
        // Test P0_14
        info!("Testing P0_14 - LED should blink");
        led2.set_high();
        Timer::after_millis(500).await;
        led2.set_low();
        Timer::after_millis(500).await;
        
        // Test P0_15
        info!("Testing P0_15 - LED should blink");
        led3.set_high();
        Timer::after_millis(500).await;
        led3.set_low();
        Timer::after_millis(500).await;
        
        // Test P0_16
        info!("Testing P0_16 - LED should blink");
        led4.set_high();
        Timer::after_millis(500).await;
        led4.set_low();
        Timer::after_millis(500).await;
        
        // Test P0_17
        info!("Testing P0_17 - LED should blink");
        led5.set_high();
        Timer::after_millis(500).await;
        led5.set_low();
        Timer::after_millis(500).await;
        
        info!("All LED tests completed, repeating...");
        Timer::after_millis(1000).await;
    }
} 