// =======================================================================================
// File: blinky.rs
//
// Copyright (c) 2025 Vanderbilt University
// Authors:
// - Tengyue Wu
// 
//
// Licensed to Vanderbilt University. All Rights Reserved.
// =======================================================================================

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
    
    // nRF52 DK LEDs: LED1=P0_17, LED2=P0_18, LED3=P0_19, LED4=P0_20
    let mut led1 = Output::new(p.P0_17, Level::Low, OutputDrive::Standard);
    let mut led2 = Output::new(p.P0_18, Level::Low, OutputDrive::Standard);
    let mut led3 = Output::new(p.P0_19, Level::Low, OutputDrive::Standard);
    let mut led4 = Output::new(p.P0_20, Level::Low, OutputDrive::Standard);

    // Wait for system to stabilize before using info!
    Timer::after_millis(500).await;
    
    info!("Starting blinky program on nRF52 DK");
    info!("LED1=P0_17, LED2=P0_18, LED3=P0_19, LED4=P0_20");

    let mut cycle = 0;
    
    loop {
        cycle += 1;
        
        // Turn on LED1
        led1.set_high();
        info!("LED1 ON (P0_17) - Cycle {}", cycle);
        Timer::after_millis(500).await;
        led1.set_low();
        Timer::after_millis(500).await;
        
        // Turn on LED2
        led2.set_high();
        info!("LED2 ON (P0_18) - Cycle {}", cycle);
        Timer::after_millis(500).await;
        led2.set_low();
        Timer::after_millis(500).await;
        
        // Turn on LED3
        led3.set_high();
        info!("LED3 ON (P0_19) - Cycle {}", cycle);
        Timer::after_millis(500).await;
        led3.set_low();
        Timer::after_millis(500).await;
        
        // Turn on LED4
        led4.set_high();
        info!("LED4 ON (P0_20) - Cycle {}", cycle);
        Timer::after_millis(500).await;
        led4.set_low();
        Timer::after_millis(500).await;
        
        // Every 5 cycles, add a summary
        if cycle % 5 == 0 {
            info!("Completed {} cycles - All LEDs working correctly", cycle);
            Timer::after_millis(1000).await;
        }
    }
}
