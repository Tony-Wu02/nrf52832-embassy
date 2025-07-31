// =======================================================================================
// File: mod.rs
//
// Copyright (c) 2025 Northwestern University
// Authors:
// - Amey Kasbe
// - Devin AI
// - Jacob Trueb
//
// Licensed to Northwestern University. All Rights Reserved.
// =======================================================================================

pub mod bma580;
pub mod bme280;
pub mod bme680;
pub mod lsm6dsl;
pub mod lsm6dso;
pub mod max14521e;
pub mod max30001;
pub mod max30102;
pub mod max86141;
pub mod npm1300;
pub mod template_driver;
pub mod tmp119;
pub mod LMP91200;

#[cfg(feature = "cat")]
pub mod cat24c32;
#[cfg(feature = "cat")]
pub mod cat_filesystem_demo;
