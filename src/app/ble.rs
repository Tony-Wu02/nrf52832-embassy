// =======================================================================================
// File: ble.rs
//
// Copyright (c) 2025 Vanderbilt University
// Authors:
// - Tengyue Wu
//
// Licensed to Vanderbilt University. All Rights Reserved.
// =======================================================================================

use crate::{
    app::{
        gatt::{BatteryServiceEvent, BioHubServiceEvent, ServerEvent},
        heartbeat::{utc_us_get, Heartbeat, HEARTBEAT_CHANNEL},
        state::*,
    },
    utils::{
        dfu::{DfuMsg, DFU_CHANNEL},
        payload_packer::SensorPayload,
    },
};

extern crate alloc;
use alloc::{collections::VecDeque, vec::Vec};
use core::{
    ptr::addr_of_mut,
    str::FromStr,
    sync::atomic::{AtomicBool, AtomicU16, Ordering::Relaxed},
};
use defmt::{error, info, trace, unwrap, warn};
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::Channel,
    pubsub::{PubSubBehavior, PubSubChannel},
};
use embassy_time::{Duration, Instant, Timer};
use futures::{
    future::{select, Either},
    pin_mut,
};
use heapless::String;
use nrf_softdevice::{
    ble::gatt_server::{self, NotifyValueError},
    RawError, Softdevice,
};
use nrf_softdevice_s132::{ble_gap_conn_params_t, ble_gap_phys_t, sd_ble_gap_phy_update, sd_ble_gap_tx_power_set, BLE_GAP_TX_POWER_ROLES_BLE_GAP_TX_POWER_ROLE_CONN};

use crate::app::heap::{MAX_HEAP_USAGE_FOR_PAYLOADS, PAYLOAD_SIZE};
#[cfg(feature = "cat")]
use crate::app::cat::{CatMsg, CAT_CHANNEL};

use thin_vec::ThinVec;

/// Reasonable constants for max buffer sizing
pub const MAX_TARGET_MTU: usize = 512;
pub const MAX_TARGET_L2CAP_MTU: usize = MAX_TARGET_MTU - 4;

///
/// Constants for BLE:
/// Choices for these values may significantly affect logical functionality and performance.
///
/// Power consumption and buffering characteristics may invaliidate the assumptions made
/// for hardware or choices for other constants.
///
/// CHOOSE WISELY:
///
pub trait BleConfig: Send + Sync + 'static {
    ///
    /// The prefix for the device name. The following 8 characters will be a hash of the device id.
    ///
    /// Maximum length is 4 characters.
    ///
    fn name_prefix(&self) -> &'static str;

    ///
    /// The firmware version string to be reported to the BLE client.
    ///
    /// Maximum length is 16 characters.
    ///
    fn firmware(&self) -> &'static str;

    ///
    /// This is the maximum transmission size configured through the Nordic SoftDevice (s132).
    /// This must be larger than the L2CAP MTU by 4 bytes minimum in order to account
    /// for the preamble bytes in the L2CAP packet.
    ///
    /// Chosen here for a reasonable maximum size that results in low fragmentation at
    /// the link layer and in th heap.
    fn mtu(&self) -> usize {
        MAX_TARGET_MTU
    }

    ///
    /// This is the maximum transmission size configured for data in the L2CAP layer.
    /// This is chosen as the maximum value that will fit in the MTU of the BLE stack.
    ///
    /// Chosen here to minimize power consumption by reducing packet overhead and maximizing
    /// compression of data per packet.
    ///
    fn l2cap_mtu(&self) -> usize {
        const L2CAP_PREABLE: usize = 4;
        assert!(self.mtu() > L2CAP_PREABLE);
        self.mtu() - L2CAP_PREABLE
    }

    /// This is the maximum number of data payloads that may be submitted to the BLE stack
    /// for unreliable notification.
    ///
    /// Chosen as a mechanism for ensuring that the BLE
    /// stack will have data available to send on the next connection interval without
    /// exposing the system to a large number of payloads that may need to be retransmitted.
    ///
    fn max_hvn_tx(&self) -> u8 {
        10
    }

    ///
    /// This is the maximum number of payloads that may be reliably retransmitted in case
    /// of a BLE disconnection near the notification attempt.
    ///
    /// Chosen here to reduce RAM constraints while still allowing for a multiple of the
    /// related to the HVN_TX queue size.
    fn max_pend_tx(&self) -> usize {
        self.max_hvn_tx() as usize * 4
    }

    ///
    /// This is the requested dBm of radio transmit power for a connection
    /// The choice of value must match data sheet options for the nRF52832 SoC
    ///
    fn conn_tx_power(&self) -> i8 {
        0
    }

    ///
    /// This is the requested minimum connection interval in 1.25ms units
    /// The choice of value must match data sheet options for the nRF52832 SoC
    ///
    /// The request must also match the guidelines for the central device
    ///
    /// Typically intervals step with 15ms and must not exceed several thresholds
    /// that you must look up in order to successfully negotiate the new connection interval.
    ///
    fn conn_interval_min(&self) -> u16 {
        12
    }

    ///
    /// This is the requested minimum connection interval in 1.25ms units
    /// The choice of value must match data sheet options for the nRF52832 SoC
    ///
    /// The request must also match the guidelines for the central device
    ///
    /// Typically intervals step with 15ms and must not exceed several thresholds
    /// that you must look up in order to successfully negotiate the new connection interval.
    ///
    fn conn_interval_max(&self) -> u16 {
        12
    }

    ///
    /// This is the number of connection events that the peripheral can skip.
    /// The choice must satisfy the equations for the connection interval and slave latency.
    /// Do the math for your central device.
    ///
    fn conn_slave_latency(&self) -> u16 {
        0
    }

    ///
    /// This is the requested timeout in 10ms units. The choice must satisfy the equations
    /// for the connection interval and slave latency. Do the math for your central device.
    ///
    /// The choice of value must match data sheet options for the nRF52832 SoC
    ///
    fn conn_timeout(&self) -> u16 {
        200
    }

    ///
    /// This is the advertisement interval in 1.25ms units.
    ///
    fn adv_interval(&self) -> u32 {
        400
    }

    ///
    /// This is the requested dBm of radio transmit power for a connection
    /// The choice of value must match data sheet options for the nRF52832 SoC
    ///
    fn adv_tx_power(&self) -> nrf_softdevice::ble::TxPower {
        nrf_softdevice::ble::TxPower::ZerodBm
    }

    ///
    /// Accepts BLE shutdown requests
    ///
    fn shutdown(&self) -> bool {
        false
    }
}

///
/// A convenience macro for defining a BLE configuration:
/// Overrride the defaults with the desired values.
///
/// ```rust
/// ble_config! {
///     name_prefix: "QSIB",
///     firmware: "v0.1.0",
///     conn_tx_power: 4,
///     conn_interval_min: 24,
///     conn_interval_max: 24,
///     adv_interval: 800,
/// }
/// ```
#[macro_export]
macro_rules! ble_config {
    (
        name_prefix: $name_prefix:expr,
        firmware: $firmware:expr,
        $(mtu: $mtu:expr,)?
        $(l2cap_mtu: $l2cap_mtu:expr,)?
        $(max_hvn_tx: $max_hvn_tx:expr,)?
        $(max_pend_tx: $max_pend_tx:expr,)?
        $(conn_tx_power: $conn_tx_power:expr,)?
        $(conn_interval_min: $conn_interval_min:expr,)?
        $(conn_interval_max: $conn_interval_max:expr,)?
        $(conn_slave_latency: $conn_slave_latency:expr,)?
        $(conn_timeout: $conn_timeout:expr,)?
        $(adv_interval: $adv_interval:expr,)?
        $(adv_tx_power: $adv_tx_power:expr,)?
        $(accept_shutdown_requests: $shutdown:expr,)?
    ) => {
        struct SensorBleConfig;
        impl qap::app::ble::BleConfig for SensorBleConfig {
            fn name_prefix(&self) -> &'static str {
                $name_prefix
            }

            fn firmware(&self) -> &'static str {
                $firmware
            }

            $(fn mtu(&self) -> usize {
                $mtu
            })?

            $(fn l2cap_mtu(&self) -> usize {
                $l2cap_mtu
            })?

            $(fn max_hvn_tx(&self) -> u8 {
                $max_hvn_tx
            })?

            $(fn max_pend_tx(&self) -> usize {
                $max_pend_tx
            })?

            $(fn conn_tx_power(&self) -> i8 {
                $conn_tx_power
            })?

            $(fn conn_interval_min(&self) -> u16 {
                $conn_interval_min
            })?

            $(fn conn_interval_max(&self) -> u16 {
                $conn_interval_max
            })?

            $(fn conn_slave_latency(&self) -> u16 {
                $conn_slave_latency
            })?

            $(fn conn_timeout(&self) -> u16 {
                $conn_timeout
            })?

            $(fn adv_interval(&self) -> u32 {
                $adv_interval
            })?

            $(fn adv_tx_power(&self) -> nrf_softdevice::ble::TxPower {
                $adv_tx_power
            })?

            $(fn shutdown(&self) -> bool {
                $shutdown
            })?
        }
        const BLE_CONF: SensorBleConfig = SensorBleConfig;
    }
}

// Channels for queuing async BLE operations
#[cfg(feature = "cat")]
pub static DEFERRED_BLE_CHANNEL: Channel<CriticalSectionRawMutex, BleMessage, 32> = Channel::new();
#[cfg(not(feature = "cat"))]
pub static DEFERRED_BLE_CHANNEL: Channel<CriticalSectionRawMutex, BleMessage, 384> = Channel::new();

pub static BLE_CHANNEL: Channel<CriticalSectionRawMutex, BleMessage, 8> = Channel::new();
pub static DYNAMIC_CONTROL_CHANNEL: PubSubChannel<CriticalSectionRawMutex, heapless::Vec<u8, 32>, 4, 4, 1> = PubSubChannel::new();

#[non_exhaustive]
pub enum BleMessage {
    StreamPayload(Payload),
    ExportPayload(ThinVec<u8>),
    ChargingStatus(bool),
    BatteryLevel(i32),
    ParamCheck,
    StateUpdate,
    #[allow(dead_code)]
    Reconnect,
}
pub struct Payload {
    pub data: ThinVec<u8>,
    #[cfg(feature = "cat")]
    pub sensor_payload: SensorPayload,
}

impl Payload {
    fn new(data: ThinVec<u8>, sensor_payload: SensorPayload) -> Result<Self, SensorPayload> {
        // Return None if oversubscribed memory usage
        let curr_heap_usage = PAYLOAD_SIZE.load(Relaxed);
        if curr_heap_usage > MAX_HEAP_USAGE_FOR_PAYLOADS as isize {
            return Err(sensor_payload);
        }

        PAYLOAD_SIZE.fetch_add(data.len() as isize, Relaxed);
        #[cfg(feature = "cat")]
        {
            Ok(Self { data, sensor_payload })
        }

        #[cfg(not(feature = "cat"))]
        {
            Ok(Self { data })
        }
    }

    /// Attempt to send wire-format data for a sensor payload over BLE
    #[cfg(not(feature = "cat"))]
    pub fn try_send(marshalled_data: ThinVec<u8>, sensor_payload: SensorPayload) {
        match Self::new(marshalled_data, sensor_payload) {
            Ok(payload) => {
                let _ = DEFERRED_BLE_CHANNEL.try_send(BleMessage::StreamPayload(payload));
            }
            Err(_sensor_payload) => {}
        }
    }

    /// Attempt to send wire-format data for a sensor payload over BLE
    /// spill to CAT when BLE is backed up
    #[cfg(feature = "cat")]
    pub fn try_send(marshalled_data: ThinVec<u8>, sensor_payload: SensorPayload) {
        use embassy_sync::channel::TrySendError;

        match SINK.load(Relaxed).into() {
            SinkState::Cat => {
                let _ = CAT_CHANNEL.try_send(CatMsg::Write(sensor_payload));
                return;
            }
            SinkState::Drop => {
                return;
            }
            _ => (),
        }

        match Self::new(marshalled_data, sensor_payload) {
            Ok(payload) => {
                if let Err(TrySendError::Full(BleMessage::StreamPayload(mut payload))) = DEFERRED_BLE_CHANNEL.try_send(BleMessage::StreamPayload(payload)) {
                    let sensor_payload = core::mem::take(&mut payload.sensor_payload);
                    let _ = CAT_CHANNEL.try_send(CatMsg::Write(sensor_payload));
                }
            }
            Err(sensor_payload) => {
                let _ = CAT_CHANNEL.try_send(CatMsg::Write(sensor_payload));
            }
        }
    }

    /// Blasting data out of CAT over BLE, never to go back to CAT
    #[cfg(feature = "cat")]
    pub async fn send(payload: ThinVec<u8>) {
        DEFERRED_BLE_CHANNEL.send(BleMessage::ExportPayload(payload)).await;
    }

    /// Reattempt sending an already allocated payload, dropping if BLE is backed up
    #[cfg(not(feature = "cat"))]
    pub fn resubmit(self) {
        let _ = DEFERRED_BLE_CHANNEL.try_send(BleMessage::StreamPayload(self));
    }

    /// Reattempt sending an already allocated payload, writing to CAT if BLE is backed up
    #[cfg(feature = "cat")]
    pub fn resubmit(self) {
        use embassy_sync::channel::TrySendError;

        if let Err(TrySendError::Full(BleMessage::StreamPayload(mut payload))) = DEFERRED_BLE_CHANNEL.try_send(BleMessage::StreamPayload(self)) {
            let sensor_payload = core::mem::take(&mut payload.sensor_payload);
            let _ = CAT_CHANNEL.try_send(CatMsg::Write(sensor_payload));
        }
    }
}

impl Drop for Payload {
    fn drop(&mut self) {
        PAYLOAD_SIZE.fetch_sub(self.data.len() as isize, Relaxed);
    }
}

fn fnv1a_hash(value: u64) -> u64 {
    let mut hash = 0xcbf29ce484222325; // FNV offset basis
    let prime = 0x100000001b3; // FNV prime

    for byte in value.to_be_bytes() {
        hash ^= byte as u64;
        hash = hash.wrapping_mul(prime);
    }

    hash
}

pub static mut DEVICE_NAME: String<12> = String::new();
/// Create a device name that is unique for the nRF52832 SoC chip.
#[allow(clippy::mut_from_ref)]
pub fn get_device_name(name_prefix: &'static str) -> &'static mut String<12> {
    assert!(name_prefix.len() <= 4, "Name prefix too long (max 4 characters)");

    // Factory Information Configuration Registers
    let ficr = nrf52832_pac::FICR::ptr();
    let device_id = unsafe { &(*ficr).deviceid };
    let lower = device_id[0].read().bits();
    let upper = device_id[1].read().bits();
    let device_id = ((upper as u64) << 32) | (lower as u64);

    // Hash the device id to get a short random string
    let device_id = fnv1a_hash(device_id);

    // Concatenate the name prefix and the device id hash
    let device_name = unsafe { &mut *addr_of_mut!(DEVICE_NAME) };
    *device_name = heapless::String::<12>::from_str(name_prefix).or(heapless::String::from_str("INVALID")).expect("Valid name");
    let mut device_hash = heapless::String::<64>::new();
    if core::fmt::write(&mut device_hash, format_args!("{:X}", device_id)).is_ok() && device_name.push_str(&device_hash[..8]).is_err() {
        error!("device_name too long");
        *device_name = heapless::String::from_str("INVALID").unwrap();
    }

    // The BLE stack can reference the static memory directly
    info!("device_name: {}", device_name.as_str());
    device_name
}

#[embassy_executor::task]
pub async fn run_deferred_ble() {
    // Feed messages from one queue to the next while we can
    loop {
        let msg = DEFERRED_BLE_CHANNEL.receive().await;
        while !is_conn_stable() {
            if SIGNAL_NOTIF.load(Relaxed) {
                Timer::after(Duration::from_millis(3)).await;
            } else {
                Timer::after(Duration::from_millis(100)).await;
            }
        }
        BLE_CHANNEL.send(msg).await;
    }
}

/// Handle BLE events
/// Advertise if not connected
#[embassy_executor::task]
pub async fn run_ble(ble: &'static dyn BleConfig) {
    let device_name = get_device_name(ble.name_prefix());

    // Configure the BLE stack for a single peripheral connection
    let config = nrf_softdevice::Config {
        // clock: Some(nrf_softdevice::raw::nrf_clock_lf_cfg_t {
        //     source: nrf_softdevice::raw::NRF_CLOCK_LF_SRC_RC as u8,
        //     accuracy: nrf_softdevice::raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
        //     rc_ctiv: 16,
        //     rc_temp_ctiv: 2,
        // }),
        clock: Some(nrf_softdevice::raw::nrf_clock_lf_cfg_t {
            source: nrf_softdevice::raw::NRF_CLOCK_LF_SRC_XTAL as u8,
            accuracy: nrf_softdevice::raw::NRF_CLOCK_LF_ACCURACY_50_PPM as u8,
            rc_ctiv: 0,
            rc_temp_ctiv: 0,
        }),
        conn_gap: Some(nrf_softdevice::raw::ble_gap_conn_cfg_t {
            conn_count: 1,
            event_length: ble.adv_interval() as u16,
        }),
        conn_gatt: Some(nrf_softdevice::raw::ble_gatt_conn_cfg_t { att_mtu: ble.mtu() as u16 }),
        conn_gatts: Some(nrf_softdevice::raw::ble_gatts_conn_cfg_t { hvn_tx_queue_size: ble.max_hvn_tx() }),
        gatts_attr_tab_size: Some(nrf_softdevice::raw::ble_gatts_cfg_attr_tab_size_t { attr_tab_size: 2048 }),
        gap_role_count: Some(nrf_softdevice::raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 1,
            central_role_count: 0,
            central_sec_count: 0,
            _bitfield_1: nrf_softdevice::raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        gap_device_name: Some(nrf_softdevice::raw::ble_gap_cfg_device_name_t {
            p_value: device_name.as_mut_ptr(),
            current_len: device_name.len() as u16,
            max_len: device_name.len() as u16,
            write_perm: unsafe { core::mem::zeroed() },
            _bitfield_1: nrf_softdevice::raw::ble_gap_cfg_device_name_t::new_bitfield_1(nrf_softdevice::raw::BLE_GATTS_VLOC_USER as u8),
        }),
        ..Default::default()
    };

    // Low priority executor: runs in thread mode, using WFE/SEV
    info!("Starting SoftDevice");
    let sd = Softdevice::enable(&config);
    let server = unwrap!(super::gatt::Server::new(sd));
    info!("Starting BLE server");

    // Runs SD in SWI0_EGU0 since it runs itself on highest priority
    let spawner = Spawner::for_current_executor().await;
    unwrap!(spawner.spawn(softdevice_task(sd)));

    // Set the firmware version
    let firmware = ble.firmware();
    assert!(firmware.len() < 16);
    let firmware_version: heapless::String<16> = heapless::String::from_str(firmware).unwrap();
    server.qsib.firmware_version_set(&firmware_version).unwrap();

    // Set the state
    let curr_state = heapless::Vec::from_slice(&[
        STATE_IDLE, 0, 0, 0, 0, 0, 0, 0, 0, // Uptime = 0
        1, 1, 1, // X, Y, Z is good ???
        0, 0, 0, 0, // reserved
    ])
    .unwrap();
    server.qsib.state_set(&curr_state).unwrap();

    #[rustfmt::skip]
    let mut adv_data = Vec::from([
        0x02, 0x01, nrf_softdevice::raw::BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE as u8,
        0x04, 0xFF, 0xFF, 0xFF, 0x18, // Manufactuerer specific data to indicate type of sensor: 0x18
        1 + device_name.len() as u8, 0x09,  // Data type indicating "Complete local name"
    ]);
    adv_data.extend(device_name.as_bytes());
    let adv_data = adv_data.as_slice();

    #[rustfmt::skip]
    let scan_data = &[
        0x11,  0x06,  // Data Type indicating "Complete list of 128-bit Service UUIDs."
        0x99, 0x78, 0x97, 0xdb, 0xf9, 0xc4, 0x39, 0x94, 0x41, 0x41, 0x9e, 0xb9, 0xc4, 0x62, 0x00, 0x00,
    ];

    let mut tx_pend: VecDeque<Payload> = VecDeque::with_capacity(ble.max_pend_tx());

    loop {
        let config = nrf_softdevice::ble::peripheral::Config {
            interval: ble.adv_interval(),
            tx_power: ble.adv_tx_power(),
            ..Default::default()
        };
        let adv = nrf_softdevice::ble::peripheral::ConnectableAdvertisement::ScannableUndirected { adv_data, scan_data };
        let conn = unwrap!(nrf_softdevice::ble::peripheral::advertise_connectable(sd, adv, &config).await);

        info!("connected, requesting connection parameters");

        if let Some(handle) = conn.handle() {
            let r = unsafe { sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLES_BLE_GAP_TX_POWER_ROLE_CONN as u8, handle, ble.conn_tx_power()) };
            warn!("sd_ble_gap_tx_power_set: {:?}", r);

            let phys = ble_gap_phys_t {
                rx_phys: nrf_softdevice::raw::BLE_GAP_PHY_2MBPS as u8,
                tx_phys: nrf_softdevice::raw::BLE_GAP_PHY_2MBPS as u8,
            };
            let r = unsafe { sd_ble_gap_phy_update(handle, &phys) };
            warn!("sd_ble_gap_phy_update: {:?}", r);
        }

        let conn_params_request = ble_gap_conn_params_t {
            min_conn_interval: ble.conn_interval_min(),
            max_conn_interval: ble.conn_interval_max(),
            slave_latency: ble.conn_slave_latency(),
            conn_sup_timeout: ble.conn_timeout(),
        };
        if let Err(err) = conn.set_conn_params(conn_params_request) {
            warn!("set_conn_params error: {:?}", err);
        }

        // Mark the connection has been reset and assume perfect time until the next connection
        critical_section::with(|_| {
            unsafe {
                CONN_INSTANT = Some(Instant::now());
            };
        });

        // Run the GATT server on the connection. This returns when the connection gets disconnected.
        //
        // Event enums (ServerEvent's) are generated by nrf_softdevice::gatt_server
        // proc macro when applied to the Server struct above
        let conn_fut = gatt_server::run(&conn, &server, |e| match e {
            ServerEvent::Bas(e) => match e {
                BatteryServiceEvent::BatteryLevelCccdWrite { notifications } => {
                    BATT_NOTIF.store(notifications, Relaxed);
                }
            },
            ServerEvent::Qsib(e) => match e {
                QsibServiceEvent::HeartbeatWrite(heartbeat) => {
                    let ticks = Instant::now().as_ticks() as i64;
                    if heartbeat.len() != 8 {
                        warn!("heartbeat len: {}", heartbeat.len());
                    }
                    let i64bytes = [heartbeat[0], heartbeat[1], heartbeat[2], heartbeat[3], heartbeat[4], heartbeat[5], heartbeat[6], heartbeat[7]];
                    let heartbeat = i64::from_le_bytes(i64bytes);
                    let _ = HEARTBEAT_CHANNEL
                        .try_send(Heartbeat {
                            ts: utc_us_get(),
                            utc_us: heartbeat,
                            soc_ticks: ticks,
                        })
                        .map_err(|_| warn!("HEARTBEAT_CHANNEL full"));
                }
                QsibServiceEvent::StateCccdWrite { notifications: _ } => {}
                QsibServiceEvent::EventWrite(val) => {
                    info!("wrote Qsib event: {:?}", val.as_slice());
                    if val.is_empty() {
                        return;
                    }

                    if val[0] == 0 {
                        // Reboot
                        warn!("rebooting");
                        cortex_m::peripheral::SCB::sys_reset();
                    }

                    if val[0] == 1 {
                        info!("start export event");
                        let _ = STATE_CHANGE_CHANNEL.try_send(STATE_EXPORT);
                    }

                    if val[0] == 5 {
                        info!("start stream event");
                        let _ = STATE_CHANGE_CHANNEL.try_send(STATE_ACTIVE);
                    }

                    if val[0] == 12 {
                        info!("toggle stream event");
                        match SINK.load(Relaxed).into() {
                            SinkState::Ble => SINK.store(SinkState::Cat.into(), Relaxed),
                            SinkState::Cat => SINK.store(SinkState::Ble.into(), Relaxed),
                            SinkState::Drop => (),
                            SinkState::Nan => (),
                        }
                    }

                    if val[0] == 13 {
                        if ble.shutdown() {
                            info!("shutdown event accepted");
                                            unsafe {
                    nrf_softdevice_s132::sd_power_system_off();
                                loop {
                                    cortex_m::asm::wfe();
                                }
                            }
                        } else {
                            info!("shutdown event rejected");
                        }
                    }

                    if val[0] == 14 {
                        info!("Setting sink event");
                        let sink_state = val[1];
                        SINK.store(sink_state, Relaxed);
                        let _ = DEFERRED_BLE_CHANNEL.try_send(BleMessage::StateUpdate);
                    }

                    if val[0] == 0xFF {
                        info!("Received {:?}", val.as_slice());
                        DYNAMIC_CONTROL_CHANNEL.publish_immediate(val);
                    }
                }
                QsibServiceEvent::EventCccdWrite { notifications } => {
                    info!("event notifications: {}", notifications);
                    EVENT_NOTIF.store(notifications, Relaxed);
                }
                QsibServiceEvent::SignalCccdWrite { notifications } => {
                    info!("signal notifications: {}", notifications);
                    SIGNAL_NOTIF.store(notifications, Relaxed);
                }
                QsibServiceEvent::DfuWrite(val) => {
                    if val == 0 {
                        let _ = DFU_CHANNEL.try_send(DfuMsg::Restart);
                        DFU.store(true, Relaxed);
                    } else if val == 1 {
                        let _ = DFU_CHANNEL.try_send(DfuMsg::Finish);
                    }
                }
                QsibServiceEvent::DfuBytesWrite(val) => {
                    let bytes: Vec<u8> = val.to_vec();
                    if DFU_CHANNEL.try_send(DfuMsg::Append(bytes.into())).is_err() {
                        let _ = DFU_CHANNEL.try_send(DfuMsg::Restart);
                    }
                }
            },
        });

        // Wait for the app to need to do something with the BLE connection
        pin_mut!(conn_fut);
        loop {
            let params = conn.conn_params();
            if DFU.load(Relaxed) {
                if params.max_conn_interval != 12 {
                    critical_section::with(|_| {
                        unsafe {
                            CONN_INSTANT = Some(Instant::now());
                        };
                    });
                    warn!("DFU: min_conn_interval: {} max_conn_interval: {}", params.min_conn_interval, params.max_conn_interval);
                    let conn_params_request = ble_gap_conn_params_t {
                        min_conn_interval: 12,
                        max_conn_interval: 12,
                        slave_latency: 0,
                        conn_sup_timeout: 200, // 2s
                    };
                    if let Err(err) = conn.set_conn_params(conn_params_request) {
                        warn!("set_conn_params error: {:?}", err);
                    }
                }
            } else if is_conn_stable() {
                match STATE.load(Relaxed) {
                    STATE_EXPORT => {
                        if params.max_conn_interval != 12 {
                            critical_section::with(|_| {
                                unsafe {
                                    CONN_INSTANT = Some(Instant::now());
                                };
                            });
                            warn!("EXPORT: min_conn_interval: {} max_conn_interval: {}", params.min_conn_interval, params.max_conn_interval);
                            let conn_params_request = ble_gap_conn_params_t {
                                min_conn_interval: 12,
                                max_conn_interval: 12,
                                slave_latency: 0,
                                conn_sup_timeout: 200, // 2s
                            };
                            if let Err(err) = conn.set_conn_params(conn_params_request) {
                                warn!("set_conn_params error: {:?}", err);
                            }
                        }
                    }
                    STATE_ACTIVE => {
                        if !(params.max_conn_interval >= ble.conn_interval_min() && params.max_conn_interval <= ble.conn_interval_max()) && params.max_conn_interval != 0 {
                            critical_section::with(|_| {
                                unsafe {
                                    CONN_INSTANT = Some(Instant::now());
                                };
                            });
                            warn!("ACTIVE: min_conn_interval: {} max_conn_interval: {}", params.min_conn_interval, params.max_conn_interval);
                            let conn_params_request = ble_gap_conn_params_t {
                                min_conn_interval: ble.conn_interval_min(),
                                max_conn_interval: ble.conn_interval_max(),
                                slave_latency: ble.conn_slave_latency(),
                                conn_sup_timeout: ble.conn_timeout(),
                            };
                            if let Err(err) = conn.set_conn_params(conn_params_request) {
                                warn!("set_conn_params error: {:?}", err);
                            }
                        }
                    }
                    STATE_IDLE | STATE_LOW_BATT => {
                        if (params.min_conn_interval < 108 && params.min_conn_interval != 0) || params.slave_latency != 12 {
                            critical_section::with(|_| {
                                unsafe {
                                    CONN_INSTANT = Some(Instant::now());
                                };
                            });
                            warn!("IDLE: min_conn_interval: {} max_conn_interval: {}", params.min_conn_interval, params.max_conn_interval);
                            let conn_params_request = ble_gap_conn_params_t {
                                min_conn_interval: 108,
                                max_conn_interval: 120,
                                slave_latency: 12,
                                conn_sup_timeout: 600, // 6s
                            };
                            if let Err(err) = conn.set_conn_params(conn_params_request) {
                                warn!("set_conn_params error: {:?}", err);
                            }
                        }
                    }
                    _ => {}
                }
            }
            let app_evt_fut = BLE_CHANNEL.receive();
            pin_mut!(app_evt_fut);
            let fut = select(conn_fut, app_evt_fut).await;
            match fut {
                Either::Left((_, _)) => {
                    // Connection closed
                    info!("disconnected");
                    break;
                }
                Either::Right((msg, left)) => {
                    conn_fut = left;
                    // App event
                    match msg {
                        BleMessage::Reconnect => {
                            info!("dropping connection");
                            let _ = conn.disconnect();
                            // Allow plenty of time for disconnection to complete
                            Timer::after(Duration::from_millis(500)).await;
                            break;
                        }
                        BleMessage::ParamCheck => {
                            let params = conn.conn_params();
                            // info!(
                            //     "min_conn_interval: {} max_conn_interval: {}",
                            //     params.min_conn_interval, params.max_conn_interval
                            // );
                            CONN_INTERVAL.store(params.max_conn_interval, Relaxed);
                        }
                        BleMessage::StateUpdate => {
                            // Set the state
                            let curr_state = STATE.load(Relaxed);
                            info!("state: {}", curr_state);
                            let curr_state = heapless::Vec::from_slice(&[
                                STATE.load(Relaxed),
                                SINK.load(Relaxed),
                                0,
                                0,
                                0,
                                0,
                                0,
                                0,
                                0, // Uptime
                                1,
                                1,
                                1, // CAT and IMU0 is good
                                0,
                                0,
                                0,
                                0,
                            ])
                            .unwrap();
                            if let Err(e) = server.qsib.state_set(&curr_state) {
                                error!("state_set error: {:?}", e);
                            }
                            // notify if subscribed
                            let _ = server.qsib.state_notify(&conn, &curr_state);
                        }
                        BleMessage::BatteryLevel(batt_mv) => {
                            if EVENT_NOTIF.load(Relaxed) {
                                let mut event = heapless::Vec::new();
                                event.extend_from_slice(&0x09_u8.to_le_bytes()).unwrap();
                                event.extend_from_slice(&batt_mv.to_le_bytes()).unwrap();
                                retry_with_backoff(4000, || server.qsib.event_notify(&conn, &event), |e| !matches!(e, NotifyValueError::Raw(RawError::Resources)))
                                    .await
                                    .map_err(|e| error!("send event notification error: {:?}", e))
                                    .ok();
                            }
                        }
                        BleMessage::ChargingStatus(is_charging) => {
                            if EVENT_NOTIF.load(Relaxed) {
                                let mut event = heapless::Vec::new();
                                event.extend_from_slice(&0x10_u8.to_le_bytes()).unwrap();
                                event.extend_from_slice(&(is_charging as u8).to_le_bytes()).unwrap();
                                retry_with_backoff(4000, || server.qsib.event_notify(&conn, &event), |e| !matches!(e, NotifyValueError::Raw(RawError::Resources)))
                                    .await
                                    .map_err(|e| error!("send event notification error: {:?}", e))
                                    .ok();
                            }
                        }
                        BleMessage::StreamPayload(stream_payload) => {
                            // Notify data over BLE or save to CAT if we can't notify
                            if SIGNAL_NOTIF.load(Relaxed) {
                                // Make sure that the payload fits in the MTU
                                let p: Result<heapless::Vec<u8, MAX_TARGET_L2CAP_MTU>, _> = stream_payload.data.as_slice().try_into();
                                let Ok(payload) = p else {
                                    error!("signal payload len: {}", stream_payload.data.len());
                                    continue;
                                };

                                // Attempt to notify the payload over BLE
                                if let Err(_e) = server.qsib.signal_notify(&conn, &payload) {
                                    // mark the connection as unstable
                                    critical_section::with(|_| {
                                        unsafe {
                                            CONN_INSTANT = Some(Instant::now());
                                        };
                                    });

                                    // try to send the payload again later
                                    warn!("Resubmitting {}", stream_payload.data.len());
                                    stream_payload.resubmit();
                                } else {
                                    // Hold a queue of recently sent payloads in case BLE disconnects
                                    if tx_pend.len() == tx_pend.capacity() {
                                        tx_pend.pop_front();
                                    }
                                    tx_pend.push_back(stream_payload);
                                }
                            } else {
                                // try to send the payload again later
                                stream_payload.resubmit();
                            }
                        }
                        BleMessage::ExportPayload(export_payload) => {
                            // Make sure that the payload fits in the MTU
                            let p: Result<heapless::Vec<u8, MAX_TARGET_L2CAP_MTU>, _> = export_payload.as_slice().try_into();
                            let Ok(payload) = p else {
                                error!("signal payload len: {}", export_payload.len());
                                continue;
                            };

                            // Retry until we can notify the payload over BLE
                            retry_with_backoff(5000, || server.qsib.signal_notify(&conn, &payload), |e| !matches!(e, NotifyValueError::Raw(RawError::Resources)))
                                .await
                                .map_err(|e| error!("send signal notification error: {:?}", e))
                                .ok();
                        }
                    }
                }
            }

            if STATE.load(Relaxed) != STATE_ACTIVE {
                // Drop pending payloads if we aren't in active mode
                tx_pend.clear();
            }
        }

        if STATE.load(Relaxed) == STATE_ACTIVE {
            for payload in tx_pend.drain(..) {
                payload.resubmit();
            }
        }

        critical_section::with(|_| unsafe { CONN_INSTANT = None });
        CONN_INTERVAL.store(0, Relaxed);
        BATT_NOTIF.store(false, Relaxed);
        EVENT_NOTIF.store(false, Relaxed);
        SIGNAL_NOTIF.store(false, Relaxed);
    }
}

async fn retry_with_backoff<F, P, A, T>(max_reattempt_delay_ms: usize, attempt: F, break_on_error: P) -> Result<A, T>
where
    F: Fn() -> Result<A, T>,
    P: Fn(T) -> bool,
    A: Copy,
    T: Copy,
{
    let mut delay_ms = 1u64;
    let max_ms = max_reattempt_delay_ms as u64;
    let mut err = None;
    let start = Instant::now();
    while delay_ms < max_ms {
        match attempt() {
            Ok(a) => return Ok(a),
            Err(e) => {
                trace!("Retry ({})", delay_ms);
                Timer::after(Duration::from_millis(delay_ms)).await;
                delay_ms *= 2;
                err = Some(e);
                if break_on_error(e) {
                    break;
                }
            }
        }
    }
    warn!("Retry failed after {}ms", start.elapsed().as_millis());
    Err(err.unwrap())
}

static mut CONN_INSTANT: Option<Instant> = None;
static CONN_INTERVAL: AtomicU16 = AtomicU16::new(0);
pub static BATT_NOTIF: AtomicBool = AtomicBool::new(false);
pub static EVENT_NOTIF: AtomicBool = AtomicBool::new(false);
pub static SIGNAL_NOTIF: AtomicBool = AtomicBool::new(false);

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

///
/// Use to check if okay to attempt to send data over BLE.
///
/// For implants, we will assume it is okay to send as soon as we have someone listening.
///
#[inline]
pub fn is_conn_stable() -> bool {
    SIGNAL_NOTIF.load(Relaxed)
        && critical_section::with(|_| unsafe {
            if let Some(instant) = CONN_INSTANT {
                instant.elapsed() > Duration::from_millis(5)
            } else {
                false
            }
        })
}
