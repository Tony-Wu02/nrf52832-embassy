// =======================================================================================
// File: pdm.rs
//
// Copyright (c) 2025 Northwestern University
// Authors:
// - Amey Kasbe
// - Jacob Trueb
//
// Licensed to Northwestern University. All Rights Reserved.
// =======================================================================================

use core::mem::MaybeUninit;
use core::ptr::addr_of_mut;

use crate::app::heartbeat::utc_us_get;
use crate::app::state::{BATTERY_LOW, STATE, STATE_ACTIVE, STATE_EXPORT};
use crate::utils::cmp_ctx::*;
use crate::utils::decimate::{Decimator, NoopDecimator};
use crate::utils::filter::{init_processor, FilterConfig};
use crate::utils::payload_packer::{CompressedBuffer, UncompressedBuffer};
use alloc::boxed::Box;
use alloc::vec::Vec;
use core::sync::atomic::Ordering::Relaxed;
use defmt::{debug, info, warn};
use embassy_nrf::pdm::{DoubleBufferSampleState, Pdm};
use embassy_nrf::peripherals;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Instant, Timer};
use fixed::types::I7F1;
use thin_vec::ThinVec;
use tsz_compress::prelude::*;

use super::ble::BleConfig;

pub trait PdmConfig: Send + Sync + 'static {
    fn sample_rate(&self) -> usize {
        let freq: usize = self.pdm_freq().into();
        let ratio: usize = self.pdm_decimation().into();
        freq / ratio
    }
    fn pdm_freq(&self) -> PdmClockFreq;
    fn pdm_decimation(&self) -> PdmDecimation;
    fn pdm_gain(&self) -> I7F1;
    fn pdm_words(&self) -> Option<usize> {
        // Default is a constant buffer size, may be required if a task relies on a constant buffer size
        None
    }
    fn z_filt(&self) -> Option<FilterConfig> {
        None
    }
    fn z_decimate(&self) -> Box<dyn Decimator> {
        Box::new(NoopDecimator)
    }
    fn z_decimation_ratio(&self) -> usize {
        1
    }
}

#[macro_export]
macro_rules! pdm_config {
    (
        pdm_freq: $pdm_freq:expr,
        pdm_ratio: $pdm_ratio:expr,
        pdm_gain: $pdm_gain:expr,
        $(pdm_words: $pdm_words:expr,)?
        $(z_filt: ($zorder:expr, $zcut:expr),)?
        $(z_decimate: ($decimation_stages:expr, $decimation_ratio:expr),)?
    ) => {
        struct PdmConfigImpl;
        impl PdmConfig for PdmConfigImpl {
            fn pdm_freq(&self) -> PdmClockFreq {
                $pdm_freq
            }
            fn pdm_decimation(&self) -> PdmDecimation {
                $pdm_ratio
            }
            fn pdm_gain(&self) -> I7F1 {
                I7F1::from_num($pdm_gain)
            }
            $(fn pdm_words(&self) -> Option<usize> {
                Some($pdm_words)
            })?
            $(fn z_filt(&self) -> Option<qap::utils::filter::FilterConfig> {
                Some(qap::utils::filter::FilterConfig {
                    order: $zorder,
                    cutoff0: $zcut,
                    cutoff1: 0.0,
                    bandtype: sci_rs::signal::filter::design::FilterBandType::Lowpass
              })
            })?
            $(
                fn z_decimate(&self) -> ::alloc::boxed::Box<dyn qap::utils::decimate::Decimator> {
                    ::alloc::boxed::Box::new(qap::utils::decimate::CicDecimator::<$decimation_stages, $decimation_ratio>::new())
                }
                fn z_decimation_ratio(&self) -> usize {
                    $decimation_ratio
                }
            )?
        }
        const PDM_CONF: PdmConfigImpl = PdmConfigImpl;
    }
}

/// The PDM clock frequency used to clock
/// in bits on the data line.
pub enum PdmClockFreq {
    _1000K = 1000000,
    _1032K = 1032000,
    _1067K = 1067000,
    _1231K = 1231000,
    _1280K = 1280000,
    _1333K = 1333000,
}

/// The decimation ratio from the PDM sampled
/// one bit at a time at PdmClockFreq to the
/// 16-bit PCM output
pub enum PdmDecimation {
    _64 = 64,
    _80 = 80,
}

impl From<PdmClockFreq> for embassy_nrf::pdm::Frequency {
    fn from(freq: PdmClockFreq) -> Self {
        match freq {
            PdmClockFreq::_1000K => embassy_nrf::pdm::Frequency::_1000K,
            PdmClockFreq::_1032K => embassy_nrf::pdm::Frequency::DEFAULT,
            PdmClockFreq::_1067K => embassy_nrf::pdm::Frequency::_1067K,
            PdmClockFreq::_1231K => embassy_nrf::pdm::Frequency::_1231K,
            PdmClockFreq::_1280K => embassy_nrf::pdm::Frequency::_1280K,
            PdmClockFreq::_1333K => embassy_nrf::pdm::Frequency::_1333K,
        }
    }
}

impl From<PdmDecimation> for embassy_nrf::pdm::Ratio {
    fn from(decimation: PdmDecimation) -> Self {
        match decimation {
            PdmDecimation::_64 => embassy_nrf::pdm::Ratio::RATIO64,
            PdmDecimation::_80 => embassy_nrf::pdm::Ratio::RATIO80,
        }
    }
}

impl From<PdmClockFreq> for usize {
    fn from(freq: PdmClockFreq) -> Self {
        match freq {
            PdmClockFreq::_1000K => 1_000_000,
            PdmClockFreq::_1032K => 1_032_000,
            PdmClockFreq::_1067K => 1_067_000,
            PdmClockFreq::_1231K => 1_231_000,
            PdmClockFreq::_1280K => 1_280_000,
            PdmClockFreq::_1333K => 1_333_000,
        }
    }
}

impl From<PdmDecimation> for usize {
    fn from(decimation: PdmDecimation) -> Self {
        match decimation {
            PdmDecimation::_64 => 64,
            PdmDecimation::_80 => 80,
        }
    }
}

pub struct PdmData {
    pub first_utc_us: i64,
    pub last_utc_us: i64,
    pub data: Vec<i16>,
}

pub mod mono {
    use tsz_compress::prelude::*;
    #[derive(Copy, Clone, CompressV2)]
    pub struct MonoRow {
        pub(crate) z: i16,
    }
}
use mono::compress::MonoRowCompressorImpl;
use mono::MonoRow;

pub mod stereo {
    use tsz_compress::prelude::*;
    #[derive(Copy, Clone, CompressV2)]
    pub struct StereoRow {
        pub(crate) z0: i16,
        pub(crate) z1: i16,
    }
}
use stereo::compress::StereoRowCompressorImpl;
use stereo::StereoRow;

pub type PdmChannel = Channel<CriticalSectionRawMutex, PdmData, 8>;
pub static BUF_QUEUE: Channel<CriticalSectionRawMutex, Vec<i16>, 4> = Channel::new();

fn alloc_buf(capacity: usize) -> (*mut i16, usize, usize) {
    let maybe_preallocated_buf = BUF_QUEUE.try_receive();
    let mut buf = if let Ok(buf) = maybe_preallocated_buf {
        buf
    } else {
        info!("Allocated new buffer");
        Vec::<i16>::with_capacity(capacity)
    };
    let ptr = buf.as_mut_ptr();
    buf.leak();
    (ptr, capacity, capacity)
}

#[embassy_executor::task]
pub async fn run_pdm(config: &'static dyn PdmConfig, mut pdm: Pdm<'static, peripherals::PDM>, channel: &'static PdmChannel) {
    let pdm_words = config.pdm_words().unwrap_or(PDM_WORDS);
    loop {
        if STATE.load(Relaxed) != STATE_ACTIVE {
            Timer::after_millis(100).await;
            continue;
        }

        // Reset processing of PDM data
        let buf_parts = alloc_buf(pdm_words);
        if channel
            .try_send(PdmData {
                first_utc_us: 0,
                last_utc_us: 0,
                data: unsafe { Vec::from_raw_parts(buf_parts.0, buf_parts.1, buf_parts.2) },
            })
            .is_err()
        {
            warn!("PDM_CHANNEL full");
        }

        let mut bufs = [alloc_buf(pdm_words), alloc_buf(pdm_words)];

        // according to nrf52 datasheet
        const PDM_DECIMATION_FILTER_DELAY_US: i64 = 5_000;
        let pdm_output_data_rate: i64 = config.sample_rate() as i64;
        let per_sample_us: i64 = 1_000_000 / pdm_output_data_rate;
        let mut last_sample_ts = utc_us_get();
        let result = pdm
            .run_double_buffered(&mut bufs, move |buf| {
                // This is the nearest estimate to the first sample in the buffer
                let ts = utc_us_get() - PDM_DECIMATION_FILTER_DELAY_US;
                let ts = (ts >> 5) << 5;

                // Bring the vec lifetime onto the stack
                let buf = unsafe { Vec::from_raw_parts(buf.0, buf.1, buf.2) };

                if BATTERY_LOW.load(Relaxed) {
                    warn!("Battery low, stopping PDM");
                    return DoubleBufferSampleState::Stop;
                }

                if STATE.load(Relaxed) == STATE_EXPORT {
                    info!("Stopping PDM for export");
                    return DoubleBufferSampleState::Stop;
                }

                // Hand off the data to a low priority task
                if channel
                    .try_send(PdmData {
                        first_utc_us: last_sample_ts + per_sample_us,
                        last_utc_us: ts,
                        data: buf,
                    })
                    .is_err()
                {
                    warn!("PDM_CHANNEL full");
                }
                last_sample_ts = ts;

                // Allocate a new buffer to write the next samples into
                let next_buf = alloc_buf(pdm_words);
                DoubleBufferSampleState::Swap(next_buf)
            })
            .await;

        // Drop the buffers that are still allocated
        for buf in bufs {
            if buf.0 as usize != 0 {
                unsafe { Vec::from_raw_parts(buf.0, buf.1, buf.2) };
            }
        }

        info!("PDM sampler stopped: {:?}", result);
    }
}

static mut MONO_COMPRESSOR: MaybeUninit<MonoRowCompressorImpl> = MaybeUninit::uninit();
static mut STEREO_COMPRESSOR: MaybeUninit<StereoRowCompressorImpl> = MaybeUninit::uninit();

///
/// Process PDM Data
///
/// Parse chip data, compress, marshal, and send to BLE
///
#[embassy_executor::task]
pub async fn run_cpu_pdm_mono(modality: u8, ble: &'static dyn BleConfig, pdm: &'static dyn PdmConfig, channel: &'static PdmChannel) {
    const ROWS_PER_PAYLOAD: usize = 200;
    let compressor = unsafe {
        MONO_COMPRESSOR.write(MonoRowCompressorImpl::new(ROWS_PER_PAYLOAD));
        MONO_COMPRESSOR.assume_init_mut()
    };
    let mut phase1 = UncompressedBuffer::new();
    let mut phase2 = CompressedBuffer::new();
    let mut ctx = MonoCompressorContext {
        modality,
        l2cap_mtu: ble.l2cap_mtu(),
        ..Default::default()
    };
    let mut last_mic = None;
    let mut mic_cpu = Instant::now().elapsed();

    let mut processor = init_processor(pdm.z_filt(), pdm.sample_rate() as f32);
    let mut decimator = pdm.z_decimate();
    let decimation_ratio = pdm.z_decimation_ratio() as f64;

    let mut first_ts = 0;

    loop {
        // Wait for the next data
        let mic = channel.receive().await;

        if mic.first_utc_us == 0 || mic.last_utc_us == 0 {
            // Reset the filter
            processor = init_processor(pdm.z_filt(), pdm.sample_rate() as f32);
            first_ts = 0;
            last_mic = None;
            mic_cpu = Instant::now().elapsed();
            *compressor = MonoRowCompressorImpl::new(ROWS_PER_PAYLOAD);
            let _ = BUF_QUEUE.try_send(mic.data);
            continue;
        }
        // Process the data
        if let Some(PdmData { first_utc_us, last_utc_us, mut data }) = last_mic {
            // timestamps are interpolated across the payload
            let elapsed = last_utc_us - first_utc_us;
            let tsd = (elapsed) as f64 / (data.len()) as f64 * decimation_ratio;

            // make sure the timestamps are valid
            if !((1577836800000000..2524608062367496).contains(&first_utc_us) && (1577836800000000..2524608062367496).contains(&last_utc_us)) || elapsed > 1_000_000 {
                last_mic = Some(mic);
                first_ts = 0;
                let _ = BUF_QUEUE.try_send(data);
                continue;
            }

            // the very first payload will use the first timestamp
            if first_ts == 0 {
                first_ts = first_utc_us;
            }

            let mut cpu_during_payload = Instant::now();

            // Compress (and filter) the data
            let filtered = processor.process16(&mut data);
            let decimated = decimator.decimate16(filtered);
            for (i, z) in decimated.iter().enumerate() {
                compressor.compress(MonoRow { z: *z });
                let rows = compressor.row_count();
                if rows >= ROWS_PER_PAYLOAD {
                    let payload_start: i64 = first_ts;
                    let payload_end: i64 = first_utc_us + (tsd * i as f64) as i64;
                    ctx.ts0 = payload_start;
                    ctx.ts1 = payload_end;
                    first_ts = payload_end + tsd as i64;

                    let payload_len = emit_payload(ble, compressor, &mut phase1, &mut phase2, &mut ctx);
                    mic_cpu += cpu_during_payload.elapsed();
                    cpu_during_payload = Instant::now();
                    debug!("MIC: {} us {} rows {} bytes {} B/row", mic_cpu.as_micros(), rows, payload_len, payload_len as f32 / rows as f32);

                    mic_cpu = Instant::now().elapsed();
                }
            }

            mic_cpu += cpu_during_payload.elapsed();
            let _ = BUF_QUEUE.try_send(data);
        }
        last_mic = Some(mic);
    }
}

pub struct StereoCicDecimator<const STAGES: usize, const RATIO: usize> {
    integrator_a: [isize; STAGES],
    integrator_b: [isize; STAGES],
    comb_a: [isize; STAGES],
    comb_b: [isize; STAGES],
    delay_idx: usize,
}

impl<const STAGES: usize, const RATIO: usize> Default for StereoCicDecimator<STAGES, RATIO> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const STAGES: usize, const RATIO: usize> StereoCicDecimator<STAGES, RATIO> {
    pub fn new() -> Self {
        const {
            // Non-zero STAGES and RATIO
            assert!(STAGES > 0);
            assert!(RATIO > 0);
        }
        Self {
            integrator_a: [0; STAGES],
            integrator_b: [0; STAGES],
            comb_a: [0; STAGES],
            comb_b: [0; STAGES],
            delay_idx: 0,
        }
    }

    #[inline(always)]
    fn decimate_stereo<'a, const N: usize, const M: usize>(&mut self, input: &[i16; N], out_a: &'a mut [i16; M], out_b: &'a mut [i16; M]) {
        if N / RATIO / 2 != M {
            panic!("N / RATIO must be equal to M");
        }

        let mut in_idx = 0;
        let mut out_idx = 0;
        while in_idx < N {
            // Load channel A
            let mut sample_a = unsafe { *input.get_unchecked(in_idx) as isize };
            in_idx += 1;
            // Load channel B
            let mut sample_b = unsafe { *input.get_unchecked(in_idx) as isize };
            in_idx += 1;

            // Integrate A and B in parallel
            self.integrator_a.iter_mut().zip(self.integrator_b.iter_mut()).for_each(|(x_a, x_b)| {
                sample_a += *x_a;
                *x_a = sample_a;
                sample_b += *x_b;
                *x_b = sample_b;
            });

            // Decimate
            self.delay_idx += 1;
            if self.delay_idx != RATIO {
                continue;
            }
            self.delay_idx = 0;

            // Comb A and B in parallel
            self.comb_a.iter_mut().zip(self.comb_b.iter_mut()).for_each(|(x_a, x_b)| {
                let zi_a = *x_a;
                let zi_b = *x_b;
                *x_a = sample_a;
                *x_b = sample_b;
                sample_a -= zi_a;
                sample_b -= zi_b;
            });

            // Normalize
            sample_a /= (RATIO.pow(STAGES as u32)) as isize;
            sample_b /= (RATIO.pow(STAGES as u32)) as isize;

            // Store
            unsafe {
                *out_a.get_unchecked_mut(out_idx) = sample_a as i16;
                *out_b.get_unchecked_mut(out_idx) = sample_b as i16;
            }
            out_idx += 1;
        }
    }
}

// Bit growth calculation for CIC decimation filter:
//
// Growth in bits = ceil(N * log2(R * M))
// Total bits = Input bits + Growth bits
//
// Where:
//   N = Number of stages
//   R = Decimation ratio
//   M = Maximum input value
//   Input bits = Initial sample bit width
//   Growth bits = Additional bits needed
//
// For current configuration:
//   Input bits = 16
//   Growth bits = 4 * log2(4) = 8
//   Total bits = 16 + 8 = 24
//
// Therefore a 32-bit accumulator provides sufficient headroom
const DECIMATION_STAGES: usize = 4;
const DECIMATION_RATIO: usize = 4;
const PDM_WORDS: usize = 256;
static mut WORDS_A: [i16; PDM_WORDS / DECIMATION_RATIO / 2] = [0; PDM_WORDS / DECIMATION_RATIO / 2];
static mut WORDS_B: [i16; PDM_WORDS / DECIMATION_RATIO / 2] = [0; PDM_WORDS / DECIMATION_RATIO / 2];

///
/// Process PDM Data
///
/// Parse chip data, compress, marshal, and send to BLE
///
#[embassy_executor::task]
pub async fn run_cpu_pdm_stereo(modality: u8, ble: &'static dyn BleConfig, pdm: &'static dyn PdmConfig, channel: &'static PdmChannel) {
    const ROWS_PER_PAYLOAD: usize = 100;
    let compressor = unsafe {
        STEREO_COMPRESSOR.write(StereoRowCompressorImpl::new(ROWS_PER_PAYLOAD));
        STEREO_COMPRESSOR.assume_init_mut()
    };
    let words_a = unsafe { &mut *addr_of_mut!(WORDS_A) };
    let words_b = unsafe { &mut *addr_of_mut!(WORDS_B) };

    assert!(pdm.pdm_words().is_none());

    let mut phase1 = UncompressedBuffer::new();
    let mut phase2 = CompressedBuffer::new();
    let mut ctx = StereoCompressorContext {
        modality,
        l2cap_mtu: ble.l2cap_mtu(),
        ..Default::default()
    };
    let mut last_mic = None;
    let mut mic_cpu = Instant::now().elapsed();

    let mut decimator: StereoCicDecimator<DECIMATION_STAGES, DECIMATION_RATIO> = StereoCicDecimator::new();

    let mut first_ts = 0;

    loop {
        // Wait for the next data
        let mic = channel.receive().await;

        if mic.first_utc_us == 0 || mic.last_utc_us == 0 {
            // Reset the filter
            first_ts = 0;
            last_mic = None;
            mic_cpu = Instant::now().elapsed();
            *compressor = StereoRowCompressorImpl::new(ROWS_PER_PAYLOAD);
            let _ = BUF_QUEUE.try_send(mic.data);
            continue;
        }
        // Process the data
        if let Some(PdmData { first_utc_us, last_utc_us, data }) = last_mic {
            // timestamps are interpolated across the payload
            let elapsed = last_utc_us - first_utc_us;
            let samples = data.len() / 2;
            let tsd = (elapsed) as f64 / samples as f64 * (DECIMATION_RATIO as f64);

            // make sure the timestamps are valid
            if !((1577836800000000..2524608062367496).contains(&first_utc_us) && (1577836800000000..2524608062367496).contains(&last_utc_us)) || elapsed > 1_000_000 {
                last_mic = Some(mic);
                first_ts = 0;
                let _ = BUF_QUEUE.try_send(data);
                continue;
            }

            // the very first payload will use the first timestamp
            if first_ts == 0 {
                first_ts = first_utc_us;
            }

            let mut cpu_during_payload = Instant::now();

            // Decimate each channel of data
            let words: &[i16; PDM_WORDS] = data.as_slice().try_into().unwrap();

            // Compress (and filter) the data
            decimator.decimate_stereo(words, words_a, words_b);
            for (i, (z0, z1)) in words_a.iter().zip(words_b.iter()).enumerate() {
                compressor.compress(StereoRow { z0: *z0, z1: *z1 });
                let rows = compressor.row_count();
                if rows >= ROWS_PER_PAYLOAD {
                    let payload_start: i64 = first_ts;
                    let payload_end: i64 = first_utc_us + (tsd * i as f64) as i64;
                    ctx.ts0 = payload_start;
                    ctx.ts1 = payload_end;
                    first_ts = payload_end + tsd as i64;

                    let payload_len = emit_payload(ble, compressor, &mut phase1, &mut phase2, &mut ctx);
                    mic_cpu += cpu_during_payload.elapsed();
                    cpu_during_payload = Instant::now();
                    debug!("MIC: {} us {} rows {} bytes {} B/row", mic_cpu.as_micros(), rows, payload_len, payload_len as f32 / rows as f32);

                    mic_cpu = Instant::now().elapsed();
                }
            }

            mic_cpu += cpu_during_payload.elapsed();
            let _ = BUF_QUEUE.try_send(data);
        }
        last_mic = Some(mic);
    }
}

#[derive(Default)]
pub struct MonoCompressorContext {
    pub modality: u8,
    pub ts0: i64,
    pub ts1: i64,
    pub l2cap_mtu: usize,
}

impl CompressorContext for MonoCompressorContext {
    fn marshal_payload(&mut self, compressor: &mut impl tsz_compress::prelude::TszCompressV2) -> ThinVec<u8> {
        let mut data = ThinVec::with_capacity(self.l2cap_mtu);
        data.extend_from_slice(&self.ts0.to_le_bytes());
        data.extend_from_slice(&self.ts1.to_le_bytes());
        compressor.finish_into_thin(&mut data);
        data
    }

    fn op_code(&self) -> u8 {
        self.modality
    }
}

#[derive(Default)]
pub struct StereoCompressorContext {
    pub modality: u8,
    pub ts0: i64,
    pub ts1: i64,
    pub l2cap_mtu: usize,
}

impl CompressorContext for StereoCompressorContext {
    fn marshal_payload(&mut self, compressor: &mut impl tsz_compress::prelude::TszCompressV2) -> ThinVec<u8> {
        let mut data = ThinVec::with_capacity(self.l2cap_mtu);
        data.extend_from_slice(&self.ts0.to_le_bytes());
        data.extend_from_slice(&self.ts1.to_le_bytes());
        compressor.finish_into_thin(&mut data);
        data
    }

    fn op_code(&self) -> u8 {
        self.modality
    }
}
