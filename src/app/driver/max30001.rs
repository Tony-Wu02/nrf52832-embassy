// =======================================================================================
// File: max30001.rs
//
// Copyright (c) 2025 Northwestern University
// Authors:
// - Amey Kasbe
// - Jacob Trueb
//
// Licensed to Northwestern University. All Rights Reserved.
// =======================================================================================

use alloc::vec;
use alloc::vec::Vec;
use core::sync::atomic::Ordering::Relaxed;
use defmt::{error, info, warn, Format};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Instant, Timer};
use thin_vec::ThinVec;
use tsz_compress::prelude::*;

use crate::app::serial_bus::SerialBus;
use crate::app::serial_bus::SerialBusError;
use crate::app::state::*;
use crate::utils::filter::init_processor;
use crate::utils::filter::FilterConfig;
use crate::InterruptPin;
use crate::{
    app::{ble::BleConfig, heartbeat::utc_us_get},
    utils::{
        cmp_ctx::{emit_payload, CompressorContext},
        payload_packer::{CompressedBuffer, UncompressedBuffer},
    },
};

pub trait Max30001Config: Send + Sync + 'static {
    fn sample_rate(&self) -> Max30001Odr;
    fn gain(&self) -> Max30001Gain;
    fn hpf(&self) -> Max30001Hpf {
        Max30001Hpf::_0_5Hz
    }
    fn lpf(&self) -> Max30001Lpf {
        Max30001Lpf::_100Hz
    }
    fn biov_filt(&self) -> Option<FilterConfig> {
        None
    }
    fn fifo_threshold(&self) -> usize {
        32
    }
    fn rows_per_payload(&self) -> usize {
        100
    }
}

///
/// Initialize the configuration of the MAX30001
/// with optional choices of filtering biov with zero, 1 or more filters
#[macro_export]
macro_rules! max30001_config {
    (
        sample_rate: $sample_rate:expr,
        gain: $gain:expr,
        $(biov_filt: ($order:expr, $cutoff0:expr, $bandtype:expr),)?
        $(hpf: $hpf:expr,)?
        $(lpf: $lpf:expr,)?
        $(fifo_threshold: $fifo_threshold:expr,)?
        $(rows_per_payload: $rows_per_payload:expr,)?
    ) => {
        struct SensorMax30001Config;
        impl Max30001Config for SensorMax30001Config {
            fn sample_rate(&self) -> Max30001Odr {
                $sample_rate
            }

            fn gain(&self) -> Max30001Gain {
                $gain
            }

            $(fn biov_filt(&self) -> Option<qap::utils::filter::FilterConfig> {
                Some(
                    qap::utils::filter::FilterConfig {
                        order: $order,
                        cutoff0: $cutoff0,
                        cutoff1: 0.0,
                        bandtype: $bandtype,
                    }
                )
            })?

            $(fn hpf(&self) -> Max30001Hpf {
                $hpf
            })?

            $(fn lpf(&self) -> Max30001Lpf {
                $lpf
            })?

            $(fn fifo_threshold(&self) -> usize {
                $fifo_threshold
            })?

            $(fn rows_per_payload(&self) -> usize {
                $rows_per_payload
            })?
        }
        const MAX30001_CONF: SensorMax30001Config = SensorMax30001Config;
    }
}

pub struct MaxMsg {
    first_utc_us: i64,
    last_utc_us: i64,
    data: Vec<u8>,
}

mod biov {
    use tsz_compress::prelude::*;
    #[derive(Copy, Clone, CompressV2)]
    pub struct BiovRow {
        pub(crate) z: i16,
    }
}
pub use biov::compress::BiovRowCompressorImpl;
use biov::BiovRow;

#[non_exhaustive]
#[derive(Clone, Copy)]
pub enum Max30001Odr {
    _256Hz,
    _512Hz,
}

#[non_exhaustive]
#[derive(Clone, Copy)]
pub enum Max30001Gain {
    _160,
    _80,
    _40,
    _20,
}

#[non_exhaustive]
#[derive(Clone, Copy)]
pub enum Max30001Hpf {
    Disabled,
    _0_5Hz,
}

impl Max30001Hpf {
    fn to_bits(self) -> u32 {
        let bits = match self {
            Max30001Hpf::Disabled => 0,
            Max30001Hpf::_0_5Hz => 1,
        };
        bits << 14
    }
}
#[non_exhaustive]
#[derive(Clone, Copy)]
pub enum Max30001Lpf {
    Disabled,
    _40Hz,
    _100Hz,
    _150Hz,
}

impl Max30001Lpf {
    fn to_bits(self) -> u32 {
        let bits = match self {
            Max30001Lpf::Disabled => 0,
            Max30001Lpf::_40Hz => 1,
            Max30001Lpf::_100Hz => 2,
            Max30001Lpf::_150Hz => 3,
        };
        bits << 12
    }
}

/// Read a register from the MAX30001
///
/// The first byte is the register address shifted up one with the read bit set
/// The next three bytes are the data: [23-16, 15-8, 7-0]
///
/// Returns the register value in little endian
///
async fn spim_read_reg(bus: &impl SerialBus, reg: u8) -> Result<u32, SerialBusError> {
    let tx = [(reg << 1) | 1];
    let mut rx = [0; 4];
    bus.xfer(&tx, &mut rx).await?;

    // Clear the first byte read during the transfer
    rx[0] = 0;

    // The data is big endian
    Ok(u32::from_be_bytes(rx))
}

/// Write a register to the MAX30001
///
/// The provided u32 will be rotated into a big endian array
/// and the bottom 24 bits will be written to the MAX30001
///
/// The first byte is the register address shifted up one
/// The next three bytes are the data: [23-16, 15-8, 7-0]
///
async fn spim_write_reg(bus: &impl SerialBus, reg: u8, val: u32) -> Result<(), SerialBusError> {
    let tx = [reg << 1, (val >> 16) as u8, (val >> 8) as u8, val as u8];
    let mut rx = [];
    bus.xfer(&tx, &mut rx).await?;
    Ok(())
}

/// There are several different FIFOs in the MAX30001
#[non_exhaustive]
enum FifoBurstReg {
    EcgFifo = 0x20,
}

/// Read the FIFO from the MAX30001
///
/// The first byte is the FIFO address shifted up one with the read bit set
/// The next N words are 3 bytes each in big endian
///
/// The output buffer should be 1 + 3 * N bytes and the first byte should be ignored
///
async fn spim_read_fifo(bus: &impl SerialBus, fifo_reg: FifoBurstReg, buf: &mut [u8]) -> Result<(), SerialBusError> {
    let tx = [(fifo_reg as u8) << 1 | 1];
    bus.xfer(&tx, buf).await?;
    Ok(())
}
async fn get_chip_id(bus: &impl SerialBus) -> Result<(u8, u8), SerialBusError> {
    let val = spim_read_reg(bus, MAX30001_INFO_REG).await?;
    let part_id = (val >> 20) & 0x0F;
    let rev_id = (val >> 16) & 0x0F;
    Ok((part_id as u8, rev_id as u8))
}

#[derive(Format)]
pub enum ChipError {
    Logic,
    Spim(embassy_nrf::spim::Error),
    SerialBus(SerialBusError),
}

impl From<embassy_nrf::spim::Error> for ChipError {
    fn from(value: embassy_nrf::spim::Error) -> Self {
        ChipError::Spim(value)
    }
}

impl From<SerialBusError> for ChipError {
    fn from(value: SerialBusError) -> Self {
        ChipError::SerialBus(value)
    }
}

// Constants according to the MAX30001 datasheet
const MAX30001_PART_ID: u8 = 0b0101;
const MAX30001_NO_OP1_REG: u8 = 0x00;
const MAX30001_STATUS_REG: u8 = 0x01;
const MAX30001_EN_INT1_REG: u8 = 0x02;
const MAX30001_MNGR_INT_REG: u8 = 0x04;
// const MAX30001_MNGR_DYN_REG: u8 = 0x05;
const MAX30001_SW_RST_REG: u8 = 0x08;
const MAX30001_SYNCH_REG: u8 = 0x09;
const MAX30001_FIFO_RST_REG: u8 = 0x0A;
const MAX30001_INFO_REG: u8 = 0x0F;
const MAX30001_CNFG_GEN_REG: u8 = 0x10;
const MAX30001_CNFG_EMUX_REG: u8 = 0x14;
const MAX30001_CNFG_ECG_REG: u8 = 0x15;

pub const MAX_BIOV_WORDS: usize = 32;

/// Bit mask
#[inline(always)]
fn bit(idx: usize) -> u32 {
    1 << idx
}

///
/// Configure ECG emux, config, interrupt, fast recovery
/// for continuous sampling
///
async fn enable(max30001: &'static dyn Max30001Config, bus: &impl SerialBus) -> Result<(), ChipError> {
    // CNFG_EMUX:
    // ECG Input Polarity Selection - Not Inverted
    // No calibration signal applied to both positive and negative
    // Internally connected input switch for both positive and negative
    const CNFG_EMUX_VAL: u32 = 0;
    spim_write_reg(bus, MAX30001_CNFG_EMUX_REG, CNFG_EMUX_VAL).await?;

    // Set ECG FIFO Interrupt Threshold (EFIT[4:0])
    // EFIT[4:0] = D[23:19] = fifo_threshold - 1 (range 0-31 maps to 1-32 unread records)
    let threshold = max30001.fifo_threshold().min(1).max(MAX_BIOV_WORDS);
    let efit = (threshold - 1) as u32;
    let efit_bits = (efit & 0x1F) << 19;
    let val = spim_read_reg(bus, MAX30001_MNGR_INT_REG).await? & !(0x1F << 19);
    spim_write_reg(bus, MAX30001_MNGR_INT_REG, val | efit_bits).await?;

    // MNGR_DYN defaults to fast recovery in normal mode

    // CNFG_ECG:
    // ECG_RATE[23:22] -> 00 = 512Hz
    // ECG_RATE[23:22] -> 01 = 256Hz
    // ECG_GAIN[17:16] -> 00 = 20/V
    // ECG_GAIN[17:16] -> 01 = 40/V
    // ECG_GAIN[17:16] -> 10 = 80/V
    // ECG_GAIN[17:16] -> 11 = 160V/V
    // ECG_DHPF[14] -> 1 = 0.5Hz Highpass enabled
    // ECG_DLPF[13:12] -> 10 = 100Hz Lowpass filter
    {
        let val = spim_read_reg(bus, MAX30001_CNFG_ECG_REG).await?;
        let val = val & !(bit(23) | bit(22) | bit(17) | bit(16) | bit(14) | bit(13) | bit(12));
        let odr_bits = match max30001.sample_rate() {
            Max30001Odr::_256Hz => bit(22),
            Max30001Odr::_512Hz => 0,
        };
        let gain_bits = match max30001.gain() {
            Max30001Gain::_160 => bit(17) | bit(16),
            Max30001Gain::_80 => bit(17),
            Max30001Gain::_40 => bit(16),
            Max30001Gain::_20 => 0,
        };
        let hpf_bits = max30001.hpf().to_bits();
        let lpf_bits = max30001.lpf().to_bits();
        // Disable highpass filter, set the gain
        let val = val | odr_bits | gain_bits | hpf_bits | lpf_bits;
        spim_write_reg(bus, MAX30001_CNFG_ECG_REG, val).await?;
    }

    // EN_INT1:
    // EN_EINT[23] -> 1 = ECG Fifo Interrupt enabled
    // INTB_TYPE[0:1] -> 11 = Open-Drain nMOS Driver with Internal 125kΩ Pullup Resistance
    // INTB_TYPE[0:1] -> 10 = Open-Drain nMOS Driver
    {
        let intb_type = bit(1) | bit(0);
        let val = bit(23) | intb_type;
        spim_write_reg(bus, MAX30001_EN_INT1_REG, val).await?;
    }

    // CNFG_GEN:
    // RBIASP[0] -> 1 = Connect resistive bias on positive
    // RBIASN[1] -> 1 = Connect resistive bias on negative
    // RBIASV[2:3] -> 00 = Resistive bias mode value is 50 MOhm
    // RBIASV[2:3] -> 01 = Resistive bias mode value is 100 MOhm
    // RBIASV[2:3] -> 10 = Resistive bias mode value is 200 MOhm
    // EN_RBIAS[4:5] -> 01 = Resistive bias enabled for ECG
    // EN_ECG[19] => 1 = Enable ECG
    {
        let val = bit(19);
        spim_write_reg(bus, MAX30001_CNFG_GEN_REG, val).await?;
    }

    // Clear PLLINT by reading STATUS
    let _ = spim_read_reg(bus, MAX30001_STATUS_REG).await?;
    Timer::after_millis(100).await;
    let _ = spim_read_reg(bus, MAX30001_STATUS_REG).await?;

    // Reset the FIFO
    spim_write_reg(bus, MAX30001_FIFO_RST_REG, 0).await?;
    spim_write_reg(bus, MAX30001_SYNCH_REG, 0).await?;

    Ok(())
}

async fn disable(bus: &impl SerialBus) -> Result<(), ChipError> {
    // CNFG_GEN:
    // EN_ECG[19] => 0 = Disable ECG
    spim_write_reg(bus, MAX30001_CNFG_GEN_REG, 0).await?;

    // EN_INT1:
    // EN_EINT[23] -> 0 = ECG Fifo Interrupt disabled
    spim_write_reg(bus, MAX30001_EN_INT1_REG, 0).await?;

    Ok(())
}

pub async fn sample_max30001(max30001: &'static dyn Max30001Config, bus: &impl SerialBus, int_pin: &mut InterruptPin, channel: &'static Max30001Channel) -> Result<(), ChipError> {
    // Chip reset
    spim_write_reg(bus, MAX30001_SW_RST_REG, 0).await?;
    let _ = spim_read_reg(bus, MAX30001_STATUS_REG).await?;
    Timer::after_millis(5).await;
    let _ = spim_read_reg(bus, MAX30001_NO_OP1_REG).await?;
    let _ = spim_read_reg(bus, MAX30001_STATUS_REG).await?;
    Timer::after_millis(5).await;

    // Confirm chip id
    let (part_id, rev_id) = get_chip_id(bus).await?;
    info!("MAX30001 Part Id {:X} Rev Id {:X}", part_id, rev_id);
    if part_id != MAX30001_PART_ID {
        error!("MAX30001 chip id {} != {}", part_id, MAX30001_PART_ID);
        Timer::after_millis(100).await;
        let (part_id, _) = get_chip_id(bus).await?;
        if part_id != MAX30001_PART_ID {
            error!("Failed to communicate with MAX30001");
            return Err(ChipError::Logic);
        }
    }

    // Turn on continuous sampling
    enable(max30001, bus).await?;

    // Some constants to applying the correct timestamp
    let output_data_rate: i64 = match max30001.sample_rate() {
        Max30001Odr::_512Hz => 512,
        Max30001Odr::_256Hz => 256,
    };
    let per_sample_delay: i64 = 1_000_000 / output_data_rate;
    let lpf_enabled = !matches!(max30001.lpf(), Max30001Lpf::Disabled);
    let filter_delay_us: i64 = match (lpf_enabled, max30001.sample_rate()) {
        (true, Max30001Odr::_256Hz) => 112610,
        (true, Max30001Odr::_512Hz) => 31555,
        (false, Max30001Odr::_256Hz) => 89172,
        (false, Max30001Odr::_512Hz) => 19836,
    };

    // As interrupts fire, drain the FIFO and hand off bytes
    let mut do_once = true;
    let mut last_sample_ts = utc_us_get();
    loop {
        if do_once {
            do_once = false;
        } else {
            int_pin.wait_for_low().await;
        }

        if STATE.load(Relaxed) != STATE_ACTIVE {
            info!("Stopping sampling");
            break;
        }

        // This is the nearest time to the interrupt that we can observe
        let utc_us = utc_us_get() - filter_delay_us;
        let mut data = MaxMsg {
            first_utc_us: last_sample_ts + per_sample_delay,
            last_utc_us: utc_us,
            data: Vec::new(),
        };
        last_sample_ts = utc_us;

        // Check the cause of the interrupt
        let status = spim_read_reg(bus, MAX30001_STATUS_REG).await?;
        let eoverflow = (status & bit(22)) != 0;
        if eoverflow {
            warn!("ECG overflowed");
            // Reset the FIFO
            spim_write_reg(bus, MAX30001_FIFO_RST_REG, 0).await?;
            spim_write_reg(bus, MAX30001_SYNCH_REG, 0).await?;
        }

        // Read the data out of the buffer
        let mut buf = vec![0u8; 1 + 3 * max30001.rows_per_payload()];
        let _ = spim_read_fifo(bus, FifoBurstReg::EcgFifo, &mut buf).await;

        // Parse the tag and reset if required
        let mut idx: usize = 3; // 1 byte throwaway, then the last byte of 1 word
        let mut overflowed = false;
        const ETAG_OVERFLOW: u8 = 0b111000;
        while idx < buf.len() {
            // Every 3 bytes, the bottom 6 bits are tag info
            let byte = unsafe { *buf.get_unchecked(idx) };
            overflowed = (byte & ETAG_OVERFLOW) == ETAG_OVERFLOW;
            idx += 3;
            if overflowed {
                break;
            }
        }

        if overflowed {
            // Reset the FIFO
            spim_write_reg(bus, MAX30001_FIFO_RST_REG, 0).await?;
            spim_write_reg(bus, MAX30001_SYNCH_REG, 0).await?;

            // Reset the processing
            data.first_utc_us = 0;
            data.last_utc_us = 0;
        }

        // Handoff this data
        data.data = buf;
        if channel.try_send(data).is_err() {
            warn!("MAX30001 Channel full");
        }
    }

    // Turn off the chip before finishing the sampling
    disable(bus).await?;

    info!("Disabled MAX30001");

    Ok(())
}

/// Macro to create tasks for MAX30001 ECG sensor
///
/// This macro generates a module with tasks and functions for managing a MAX30001 ECG sensor instance.
/// It creates a channel for communication, async tasks for running the sensor, and a spawn function.
///
/// # Arguments
///
/// * `$inst` - The name of the instance (e.g., inst0, inst1)
/// * `$bus` - The bus type used for communication (e.g., SPIM0, SPIM1)
#[macro_export]
macro_rules! create_max30001_tasks {
    ($inst:ident, $bus:ty) => {
        pub mod $inst {
            use super::*;
            use core::mem::MaybeUninit;
            use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
            use embassy_sync::channel::Channel;
            use tsz_compress::prelude::*;

            use qap::app::serial_bus::SerialBus;
            use qap::drivers::max30001::BiovRowCompressorImpl;
            use qap::drivers::max30001::Max30001Channel;
            use qap::*;

            /// Channel for communicating MAX30001 sensor data
            static CHANNEL: Max30001Channel = Channel::new();

            /// Static memory for the compressor, they use a lot of memory
            static mut COMPRESSOR: MaybeUninit<BiovRowCompressorImpl> = MaybeUninit::uninit();

            static mut RAW_WORDS: [isize; qap::drivers::max30001::MAX_BIOV_WORDS] = [0isize; qap::drivers::max30001::MAX_BIOV_WORDS];

            /// Async task for running the MAX30001 sensor
            #[embassy_executor::task]
            async fn run(max30001: &'static dyn Max30001Config, bus: $bus, mut int_pin: InterruptPin) {
                qap::drivers::max30001::run_max30001(max30001, bus, int_pin, &CHANNEL).await;
            }

            /// Async task for running the MAX30001 sensor in blocking mode
            #[embassy_executor::task]
            async fn run_blocking(modality: u8, ble: &'static dyn BleConfig, max30001: &'static dyn Max30001Config) {
                unsafe {
                    COMPRESSOR.write(BiovRowCompressorImpl::new(1));
                    let compressor = COMPRESSOR.assume_init_mut();
                    qap::drivers::max30001::run_blocking_max30001(modality, ble, max30001, &CHANNEL, compressor, &mut RAW_WORDS).await;
                }
            }

            /// Spawn function to start MAX30001 tasks
            pub fn spawn(modality: u8, spawners: &Spawners, ble: &'static dyn BleConfig, max30001: &'static dyn Max30001Config, bus: $bus, int_pin: InterruptPin) {
                spawners.spawn(NonBlocking, $inst::run(max30001, bus, int_pin));
                spawners.spawn(Blocking, $inst::run_blocking(modality, ble, max30001));
            }
        }
    };
}

pub type Max30001Channel = Channel<CriticalSectionRawMutex, MaxMsg, 8>;

pub async fn run_max30001(max30001: &'static dyn Max30001Config, bus: impl SerialBus, mut int_pin: InterruptPin, channel: &'static Max30001Channel) {
    let bus = &bus;
    loop {
        if STATE.load(Relaxed) != STATE_ACTIVE {
            Timer::after_millis(100).await;
            continue;
        }

        if channel
            .try_send(MaxMsg {
                first_utc_us: 0,
                last_utc_us: 0,
                data: Vec::new(),
            })
            .is_err()
        {
            warn!("MAX channel full");
        }

        match sample_max30001(max30001, bus, &mut int_pin, channel).await {
            Ok(_) => info!("MAX30001 sample success"),
            Err(e) => {
                warn!("MAX30001 sample failed: {:?}", e);
                Timer::after_secs(1).await;
            }
        }
    }
}

///
/// Process Max30001 data
///
/// Parse chip data, compress, marshal, and send to BLE
///
pub async fn run_blocking_max30001(
    modality: u8, ble: &'static dyn BleConfig, max30001: &'static dyn Max30001Config, channel: &'static Max30001Channel, compressor: &'static mut BiovRowCompressorImpl,
    raw_words: &'static mut [isize],
) {
    let rows_per_payload = max30001.rows_per_payload();
    let mut phase1 = UncompressedBuffer::new();
    let mut phase2 = CompressedBuffer::new();
    let mut ctx = Max30001CompressorContext {
        modality,
        l2cap_mtu: ble.l2cap_mtu(),
        ..Default::default()
    };

    let mut last_maxim = None;
    let mut maxim_cpu = Instant::now().elapsed();

    let hz = match max30001.sample_rate() {
        Max30001Odr::_256Hz => 256.0,
        Max30001Odr::_512Hz => 512.0,
    };
    let mut processor = init_processor(max30001.biov_filt(), hz);

    // Scale the i18 data down to i16 data
    let to_i16 = |i: isize| {
        let scaled_sample = i / 2;
        scaled_sample.clamp(i16::MIN as isize, i16::MAX as isize) as i16
    };

    let mut first_ts = 0;

    loop {
        // Wait for the next data
        let maxim = channel.receive().await;

        if maxim.first_utc_us == 0 || maxim.last_utc_us == 0 {
            // Reset the filter
            processor = init_processor(max30001.biov_filt(), hz);
            first_ts = 0;
            last_maxim = None;
            maxim_cpu = Instant::now().elapsed();
            *compressor = BiovRowCompressorImpl::new(rows_per_payload);
            continue;
        }

        // Process the data
        if let Some(MaxMsg { first_utc_us, last_utc_us, data }) = last_maxim {
            // timestamps are interpolated across the payload
            let elapsed = last_utc_us - first_utc_us;
            let words = (data.len() - 1) / 3;
            let tsd = (elapsed) as f64 / words as f64;

            // make sure the timestamps are valid
            if !((1577836800000000..2524608062367496).contains(&first_utc_us) && (1577836800000000..2524608062367496).contains(&last_utc_us)) || elapsed > 1_000_000 {
                last_maxim = Some(maxim);
                first_ts = 0;
                continue;
            }

            // the very first payload will use the first timestamp
            if first_ts == 0 {
                first_ts = first_utc_us;
            }

            let mut cpu_during_payload = Instant::now();

            // Parse the tags to get data
            const ETAG_MASK: u8 = 0b111000;
            const ETAG_VALID: u8 = 0b000000;
            const ETAG_LAST_VALID: u8 = 0b010000;
            let mut idx: usize = 1; // 1 byte throwaway, then the last byte of 1 word
            let mut words: usize = 0;
            while idx < data.len() {
                // Every 3 bytes, we have 1 word
                // The top 18 bits are two's complement
                // The bottom 6 bits are tag info

                // bits 1-0, etag 3 bits, ptag 3 bits
                let b2 = unsafe { *data.get_unchecked(idx + 2) };
                let tags = b2 & ETAG_MASK;
                let valid = (tags == ETAG_VALID) || (tags == ETAG_LAST_VALID);
                if valid {
                    // bits 18-11
                    let b0 = unsafe { *data.get_unchecked(idx) };
                    // bits 11-2
                    let b1 = unsafe { *data.get_unchecked(idx + 1) };
                    // sign extend for 18 bit two's complement
                    let bs = if (b0 & bit(7) as u8) != 0 { 0xFFu8 } else { 0x00u8 };
                    let bytes = [bs, b0, b1, b2];
                    let i18 = i32::from_be_bytes(bytes) >> 6;

                    unsafe {
                        *raw_words.get_unchecked_mut(words) = i18 as isize;
                    };
                    words += 1;
                }
                idx += 3;
            }
            let raw_words = &mut raw_words[..words];

            // Compress the data
            let zf = processor.process32(raw_words);
            for (i, z) in zf.iter().copied().map(to_i16).enumerate() {
                let row = BiovRow { z };
                compressor.compress(row);
                let rows = compressor.row_count();
                if rows == rows_per_payload {
                    ctx.ts0 = first_ts;
                    ctx.ts1 = first_utc_us + (tsd * i as f64) as i64;
                    first_ts = ctx.ts1 + tsd as i64;
                    let payload_len = emit_payload(ble, compressor, &mut phase1, &mut phase2, &mut ctx);

                    maxim_cpu += cpu_during_payload.elapsed();
                    cpu_during_payload = Instant::now();
                    info!(
                        "MAX: {} us {} rows {} bytes {} B/row ({})",
                        maxim_cpu.as_micros(),
                        rows,
                        payload_len,
                        payload_len as f32 / rows as f32,
                        z,
                    );

                    maxim_cpu = Instant::now().elapsed();
                }
            }

            maxim_cpu += cpu_during_payload.elapsed();
        }
        last_maxim = Some(maxim);
    }
}

#[derive(Default)]
struct Max30001CompressorContext {
    modality: u8,
    ts0: i64,
    ts1: i64,
    l2cap_mtu: usize,
}

impl CompressorContext for Max30001CompressorContext {
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
