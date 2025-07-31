// =======================================================================================
// File: gatt.rs
//
// Copyright (c) 2025 Northwestern University
// Authors:
// - Jacob Trueb
//
// Licensed to Northwestern University. All Rights Reserved.
// =======================================================================================

use super::ble::MAX_TARGET_L2CAP_MTU;

#[nrf_softdevice::gatt_service(uuid = "180f")]
pub struct BatteryService {
    #[characteristic(uuid = "2a19", read, notify)]
    battery_level: u8,
}

#[nrf_softdevice::gatt_service(uuid = "000062c4-b99e-4141-9439-c4f9db977899")]
pub struct BioHubService {
    #[characteristic(uuid = "010062c4-b99e-4141-9439-c4f9db977899", read, write)]
    pub heartbeat: heapless::Vec<u8, 16>,
    #[characteristic(uuid = "020062c4-b99e-4141-9439-c4f9db977899", read, notify)]
    pub signal: heapless::Vec<u8, MAX_TARGET_L2CAP_MTU>,
    #[characteristic(uuid = "030062c4-b99e-4141-9439-c4f9db977899", read)]
    pub firmware_version: heapless::String<16>,
    #[characteristic(uuid = "110062c4-b99e-4141-9439-c4f9db977899", read, notify)]
    pub state: heapless::Vec<u8, 16>,
    #[characteristic(uuid = "120062c4-b99e-4141-9439-c4f9db977899", read, write, notify)]
    pub event: heapless::Vec<u8, 32>,
    #[characteristic(uuid = "FE0062c4-b99e-4141-9439-c4f9db977899", write)]
    pub dfu: u8,
    #[characteristic(uuid = "FF0062c4-b99e-4141-9439-c4f9db977899", write)]
    pub dfu_bytes: heapless::Vec<u8, 110>,
}

#[nrf_softdevice::gatt_server]
pub struct Server {
    pub bas: BatteryService,
    pub qsib: QsibService,
}
