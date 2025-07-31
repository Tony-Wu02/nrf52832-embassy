// =======================================================================================
// File: cat.rs
//
// Copyright (c) 2025 Vanderbilt University
// Authors:
// - Tengyue Wu
//
// Licensed to Vanderbilt University. All Rights Reserved.
// =======================================================================================

use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use crate::utils::payload_packer::SensorPayload;

// CAT channel for communication
pub static CAT_CHANNEL: Channel<CriticalSectionRawMutex, CatMsg, 32> = Channel::new();

// CAT message types
pub enum CatMsg {
    Write(SensorPayload),
    Read,
    Erase,
}

// Re-export CAT modules from driver directory
pub use crate::app::driver::cat24c32;
pub use crate::app::driver::cat_filesystem_demo; 