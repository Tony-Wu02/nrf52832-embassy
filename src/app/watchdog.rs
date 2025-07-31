// =======================================================================================
// File: watchdog.rs
//
// Copyright (c) 2025 Northwestern University
// Authors:
// - Jacob Trueb
//
// Licensed to Northwestern University. All Rights Reserved.
// =======================================================================================

use embassy_nrf::wdt::WatchdogHandle;
use embassy_time::{Duration, Ticker};

/// Prevent the bootloader from rebooting the device
#[embassy_executor::task]
pub async fn run_watchdog(mut wdt_handle: WatchdogHandle) {
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        wdt_handle.pet();
        ticker.next().await;
    }
}
