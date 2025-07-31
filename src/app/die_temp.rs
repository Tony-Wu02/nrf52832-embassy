// =======================================================================================
// File: die_temp.rs
//
// Copyright (c) 2025 Northwestern University
// Authors:
// - Jacob Trueb
// - Josh Prunty
//
// Licensed to Northwestern University. All Rights Reserved.
// =======================================================================================

use core::mem::MaybeUninit;
use core::sync::atomic::Ordering::Relaxed;
use defmt::{debug, trace};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Ticker};
use nrf_softdevice_s132::{sd_temp_get, NRF_SUCCESS};
use thin_vec::ThinVec;
use tsz_compress::prelude::*;

use crate::{
    app::{
        ble::{is_conn_stable, BleConfig},
        heartbeat::utc_us_get,
        state::*,
    },
    utils::{
        cmp_ctx::{emit_payload, CompressorContext},
        payload_packer::{CompressedBuffer, UncompressedBuffer},
    },
};

mod temp {
    use super::*;

    #[allow(dead_code)]
    #[derive(Copy, Clone, CompressV2)]
    pub struct Row {
        #[tsz(delta = "i32")]
        pub ts: i64,
        pub internal: i16,
    }
}
use temp::compress::RowCompressorImpl;
use temp::Row;

struct TemperatureCompressorContext;

impl CompressorContext for TemperatureCompressorContext {
    fn marshal_payload(&mut self, compressor: &mut impl tsz_compress::prelude::TszCompressV2) -> ThinVec<u8> {
        compressor.finish_thin()
    }

    fn op_code(&self) -> u8 {
        0x01
    }
}

static mut COMPRESSOR: MaybeUninit<RowCompressorImpl> = MaybeUninit::uninit();

static CHANNEL: Channel<CriticalSectionRawMutex, DieTempMsg, 2> = Channel::new();

#[repr(packed)]
struct DieTempMsg {
    ts: i64,
    temp: i16,
}

#[embassy_executor::task]
pub async fn run_die_temp() {
    let idle_duration = Duration::from_secs(60);
    let active_duration = Duration::from_secs(10);
    let other_duration = Duration::from_secs(5);
    let mut curr_duration = idle_duration;
    let mut ticker = Ticker::every(curr_duration);
    loop {
        ticker.next().await;

        // During active measurements, check temperature every 10 seconds
        // During idle, check the temperature every minute
        // Do not check the temperature otherwise
        match STATE.load(Relaxed) {
            STATE_IDLE => {
                if curr_duration != idle_duration {
                    curr_duration = idle_duration;
                    ticker = Ticker::every(curr_duration);
                }
            }
            STATE_ACTIVE => {
                if curr_duration != active_duration {
                    curr_duration = active_duration;
                    ticker = Ticker::every(curr_duration);
                }
            }
            _ => {
                if curr_duration != other_duration {
                    curr_duration = other_duration;
                    ticker = Ticker::every(curr_duration);
                }
                continue;
            }
        }

        // enable the high frequency clock for a bit to take a die temperature reading
        let r = unsafe { &*nrf52832_pac::CLOCK::ptr() };
        let enable_hfclk = r.hfclkstat.read().state().is_not_running();
        if enable_hfclk {
            r.events_hfclkstarted.write(|w| unsafe { w.bits(0) });
            r.tasks_hfclkstart.write(|w| unsafe { w.bits(1) });
            while r.events_hfclkstarted.read().bits() == 0 {}
        }

        // Read the temperature without disturbing the SoftDevice clock usage
        let ts = utc_us_get() / 1000;
        let mut temp: i32 = 0;
        critical_section::with(|_| while unsafe { sd_temp_get(&mut temp) } != NRF_SUCCESS {});

        // disable the high frequency clock
        if enable_hfclk {
            r.tasks_hfclkstop.write(|w| unsafe { w.bits(1) });
        }

        // Send to the low priority task
        let msg = DieTempMsg { ts, temp: (temp as i16) };
        let _ = CHANNEL.try_send(msg);
    }
}

#[embassy_executor::task]
pub async fn run_cpu_die_temp(ble: &'static dyn BleConfig) {
    const ROWS_PER_PAYLOAD: usize = 120;
    let compressor = unsafe {
        COMPRESSOR.write(RowCompressorImpl::new(ROWS_PER_PAYLOAD));
        COMPRESSOR.assume_init_mut()
    };
    let mut phase1 = UncompressedBuffer::new();
    let mut phase2 = CompressedBuffer::new();
    let mut ctx = TemperatureCompressorContext;
    loop {
        let DieTempMsg { ts, temp } = CHANNEL.receive().await;

        // Log temperature in Celsius
        let temp_c = temp / 4;
        let frac_str = ["00", "25", "50", "75"][(temp % 4) as usize];
        debug!("CPU: {}.{}°C", temp_c, frac_str);

        if !(1577836800000..2524608062367).contains(&ts) {
            trace!("invalid ts: {}", ts);
            continue;
        }

        compressor.compress(Row { ts, internal: temp });
        if is_conn_stable() || compressor.row_count() == ROWS_PER_PAYLOAD {
            emit_payload(ble, compressor, &mut phase1, &mut phase2, &mut ctx);
        }
    }
}
