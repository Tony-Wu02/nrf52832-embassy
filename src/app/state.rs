// =======================================================================================
// File: state.rs
//
// Copyright (c) 2025 Northwestern University
// Authors:
// - Amey Kasbe
// - Jacob Trueb
//
// Licensed to Northwestern University. All Rights Reserved.
// =======================================================================================

// Control the logical application state

use crate::app::ble::*;

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering::Relaxed};
use defmt::info;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};

// State machine
pub const STATE_LOW_BATT: u8 = 2;
pub const STATE_IDLE: u8 = 3;
pub const STATE_ACTIVE: u8 = 4;
pub const STATE_EXPORT: u8 = 5;
pub static STATE: AtomicU8 = AtomicU8::new(STATE_IDLE);
pub static STATE_CHANGE_CHANNEL: Channel<CriticalSectionRawMutex, u8, 8> = Channel::new();

pub enum SinkState {
    Ble,
    Cat,
    Drop,
    Nan,
}

impl From<u8> for SinkState {
    fn from(value: u8) -> Self {
        match value {
            0 => SinkState::Ble,
            1 => SinkState::Cat,
            2 => SinkState::Drop,
            _ => SinkState::Nan,
        }
    }
}

impl From<SinkState> for u8 {
    fn from(value: SinkState) -> Self {
        match value {
            SinkState::Ble => 0,
            SinkState::Cat => 1,
            SinkState::Drop => 2,
            SinkState::Nan => 3,
        }
    }
}

// Control the state of the device
pub static DFU: AtomicBool = AtomicBool::new(false);
pub static SINK: AtomicU8 = AtomicU8::new(2);

#[allow(dead_code)]
pub static BATTERY_LOW: AtomicBool = AtomicBool::new(false);

pub enum StateChangeError {
    SetupTeardownError,
}

pub async fn nop_setup(_: u8) -> Result<(), StateChangeError> {
    Ok(())
}

pub async fn nop_teardown(_: u8) -> Result<(), StateChangeError> {
    Ok(())
}

pub async fn handle_state_change<SF, SFut, TF, TFut>(setup: SF, teardown: TF) -> Result<(), StateChangeError>
where
    SF: FnOnce(u8) -> SFut,
    SFut: core::future::Future<Output = Result<(), StateChangeError>>,
    TF: FnOnce(u8) -> TFut,
    TFut: core::future::Future<Output = Result<(), StateChangeError>>,
{
    // Receive the desired state change
    let start_state = STATE_CHANGE_CHANNEL.receive().await;

    // Perform the setup/teardown like reconfiguring GPIOs or other peripherals
    setup(start_state).await?;

    // Check if we would start exporting if we had CAT, but we don't
    if start_state == STATE_EXPORT && STATE.load(Relaxed) != STATE_EXPORT {
        #[cfg(not(feature = "cat"))]
        {
            info!("Skipping exporting, transitioning to idling");
            STATE_CHANGE_CHANNEL.send(STATE_IDLE).await;
        }
        #[cfg(feature = "cat")]
        {
            use crate::app::cat::{CatMsg, CAT_CHANNEL};

            info!("Starting exporting");
            STATE.store(STATE_EXPORT, Relaxed);
            SINK.store(SinkState::Ble.into(), Relaxed);
            DEFERRED_BLE_CHANNEL.send(BleMessage::StateUpdate).await;
            CAT_CHANNEL.send(CatMsg::Read).await;
        }
    }

    // Check if we should start streaming
    if start_state == STATE_ACTIVE && STATE.load(Relaxed) != STATE_ACTIVE {
        info!("Starting streaming");
        STATE.store(STATE_ACTIVE, Relaxed);
        SINK.store(SinkState::Ble.into(), Relaxed);
        DEFERRED_BLE_CHANNEL.send(BleMessage::StateUpdate).await;
        #[cfg(feature = "cat")]
        {
            use crate::app::cat::{CatMsg, CAT_CHANNEL};
            CAT_CHANNEL.send(CatMsg::Erase).await;
        }
    }

    // Check if we are done exporting
    if start_state == STATE_IDLE && STATE.load(Relaxed) != STATE_IDLE {
        info!("Starting idling");
        STATE.store(STATE_IDLE, Relaxed); // IDLE state
        SINK.store(SinkState::Drop.into(), Relaxed);
        DEFERRED_BLE_CHANNEL.send(BleMessage::StateUpdate).await;
    }

    // Check if we need to enter low power mode
    if start_state == STATE_LOW_BATT && STATE.load(Relaxed) != STATE_LOW_BATT {
        STATE.store(STATE_LOW_BATT, Relaxed);
        SINK.store(SinkState::Drop.into(), Relaxed);
        DEFERRED_BLE_CHANNEL.send(BleMessage::StateUpdate).await;
    }

    // Perform the teardown
    teardown(start_state).await?;

    Ok(())
}
