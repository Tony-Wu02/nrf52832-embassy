// =======================================================================================
// File: mod.rs
//
// Copyright (c) 2025 Northwestern University
// Authors:
// - Jacob Trueb
// - Josh Prunty
//
// Licensed to Northwestern University. All Rights Reserved.
// =======================================================================================

pub mod ble;
pub mod gatt;
pub mod heap;
pub mod heartbeat;
pub mod pdm;
pub mod priority;
pub mod serial_bus;
pub mod state;
pub mod watchdog;

#[cfg(not(feature = "no-soc-die-temp"))]
pub mod die_temp;

#[cfg(feature = "cat")]
pub mod cat;
