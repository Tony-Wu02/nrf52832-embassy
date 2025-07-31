// =======================================================================================
// File: LMP91200.rs
//
// Copyright (c) 2025 Vanderbilt University
// Authors:
// - Tengyue Wu
//
// Licensed to Vanderbilt University. All Rights Reserved.
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
use crate::InterruptPin;
use crate::{
    app::{ble::BleConfig, heartbeat::utc_us_get},
    utils::{
        cmp_ctx::{emit_payload, CompressorContext},
        payload_packer::{CompressedBuffer, UncompressedBuffer},
    },
};

pub trait Lmp91200Config: Send + Sync + 'static {
    fn sample_rate(&self) -> Lmp91200Odr;
    fn gain(&self) -> Lmp91200Gain;
    fn reference_voltage(&self) -> Lmp91200Vref;
    fn temperature_compensation(&self) -> bool;
    fn calibration_enabled(&self) -> bool;
    fn fifo_threshold(&self) -> usize {
        32
    }
    fn rows_per_payload(&self) -> usize {
        100
    }
}

///
/// Initialize the configuration of the LMP91200
/// with optional choices of filtering pH data
#[macro_export]
macro_rules! lmp91200_config {
    (
        sample_rate: $sample_rate:expr,
        gain: $gain:expr,
        reference_voltage: $reference_voltage:expr,
        $(temperature_compensation: $temp_comp:expr,)?
        $(calibration_enabled: $cal_enabled:expr,)?
        $(fifo_threshold: $fifo_threshold:expr,)?
        $(rows_per_payload: $rows_per_payload:expr,)?
    ) => {
        struct SensorLmp91200Config;
        impl Lmp91200Config for SensorLmp91200Config {
            fn sample_rate(&self) -> Lmp91200Odr {
                $sample_rate
            }

            fn gain(&self) -> Lmp91200Gain {
                $gain
            }

            fn reference_voltage(&self) -> Lmp91200Vref {
                $reference_voltage
            }

            $(fn temperature_compensation(&self) -> bool {
                $temp_comp
            })?

            $(fn calibration_enabled(&self) -> bool {
                $cal_enabled
            })?

            $(fn fifo_threshold(&self) -> usize {
                $fifo_threshold
            })?

            $(fn rows_per_payload(&self) -> usize {
                $rows_per_payload
            })?
        }
        const LMP91200_CONF: SensorLmp91200Config = SensorLmp91200Config;
    }
}

pub struct LmpMsg {
    first_utc_us: i64,
    last_utc_us: i64,
    data: Vec<u8>,
}

mod ph {
    use tsz_compress::prelude::*;
    #[derive(Copy, Clone, CompressV2)]
    pub struct PhRow {
        pub(crate) ph: i16,
        pub(crate) temperature: i16,
        pub(crate) voltage: i16,
    }
}
pub use ph::compress::PhRowCompressorImpl;
use ph::PhRow;

#[non_exhaustive]
#[derive(Clone, Copy)]
pub enum Lmp91200Odr {
    _1Hz,
    _2Hz,
    _4Hz,
    _8Hz,
    _16Hz,
    _32Hz,
}

#[non_exhaustive]
#[derive(Clone, Copy)]
pub enum Lmp91200Gain {
    _1,
    _2,
    _4,
    _8,
    _16,
    _32,
}

#[non_exhaustive]
#[derive(Clone, Copy)]
pub enum Lmp91200Vref {
    _2_5V,
    _3_3V,
    _5V,
}

impl Lmp91200Vref {
    fn to_voltage(self) -> f32 {
        match self {
            Lmp91200Vref::_2_5V => 2.5,
            Lmp91200Vref::_3_3V => 3.3,
            Lmp91200Vref::_5V => 5.0,
        }
    }
}

/// Read a register from the LMP91200 via I2C
async fn i2c_read_reg(bus: &impl SerialBus, reg: u8) -> Result<u16, SerialBusError> {
    let mut rx = [0u8; 2];
    bus.write_read(0x90, &[reg], &mut rx).await?;
    Ok(u16::from_be_bytes(rx))
}

/// Write a register to the LMP91200 via I2C
async fn i2c_write_reg(bus: &impl SerialBus, reg: u8, val: u16) -> Result<(), SerialBusError> {
    let data = [reg, (val >> 8) as u8, val as u8];
    bus.write(0x90, &data).await?;
    Ok(())
}

/// Read multiple registers from the LMP91200
async fn i2c_read_regs(bus: &impl SerialBus, reg: u8, buf: &mut [u8]) -> Result<(), SerialBusError> {
    bus.write_read(0x90, &[reg], buf).await?;
    Ok(())
}

async fn get_chip_id(bus: &impl SerialBus) -> Result<u16, SerialBusError> {
    let val = i2c_read_reg(bus, LMP91200_DEVICE_ID_REG).await?;
    Ok(val)
}

#[derive(Format)]
pub enum ChipError {
    Logic,
    I2c(embassy_nrf::twim::Error),
    SerialBus(SerialBusError),
}

impl From<embassy_nrf::twim::Error> for ChipError {
    fn from(value: embassy_nrf::twim::Error) -> Self {
        ChipError::I2c(value)
    }
}

impl From<SerialBusError> for ChipError {
    fn from(value: SerialBusError) -> Self {
        ChipError::SerialBus(value)
    }
}

// Constants according to the LMP91200 datasheet
const LMP91200_DEVICE_ID: u16 = 0x0001; // Expected device ID
const LMP91200_DEVICE_ID_REG: u8 = 0x00;
const LMP91200_STATUS_REG: u8 = 0x01;
const LMP91200_CONFIG_REG: u8 = 0x02;
const LMP91200_GAIN_REG: u8 = 0x03;
const LMP91200_REF_VOLTAGE_REG: u8 = 0x04;
const LMP91200_TEMP_COMP_REG: u8 = 0x05;
const LMP91200_CALIBRATION_REG: u8 = 0x06;
const LMP91200_DATA_REG: u8 = 0x07;
const LMP91200_TEMP_REG: u8 = 0x08;

pub const MAX_PH_WORDS: usize = 32;

/// Bit mask
#[inline(always)]
fn bit(idx: usize) -> u16 {
    1 << idx
}

///
/// Configure LMP91200 for pH sensing
/// Set gain, reference voltage, temperature compensation, and calibration
///
async fn enable(lmp91200: &'static dyn Lmp91200Config, bus: &impl SerialBus) -> Result<(), ChipError> {
    // CONFIG_REG: Configure the main settings
    let config_val = match lmp91200.sample_rate() {
        Lmp91200Odr::_1Hz => 0x00,
        Lmp91200Odr::_2Hz => 0x01,
        Lmp91200Odr::_4Hz => 0x02,
        Lmp91200Odr::_8Hz => 0x03,
        Lmp91200Odr::_16Hz => 0x04,
        Lmp91200Odr::_32Hz => 0x05,
    };
    i2c_write_reg(bus, LMP91200_CONFIG_REG, config_val).await?;

    // GAIN_REG: Set the amplifier gain
    let gain_val = match lmp91200.gain() {
        Lmp91200Gain::_1 => 0x00,
        Lmp91200Gain::_2 => 0x01,
        Lmp91200Gain::_4 => 0x02,
        Lmp91200Gain::_8 => 0x03,
        Lmp91200Gain::_16 => 0x04,
        Lmp91200Gain::_32 => 0x05,
    };
    i2c_write_reg(bus, LMP91200_GAIN_REG, gain_val).await?;

    // REF_VOLTAGE_REG: Set reference voltage
    let ref_voltage_val = match lmp91200.reference_voltage() {
        Lmp91200Vref::_2_5V => 0x00,
        Lmp91200Vref::_3_3V => 0x01,
        Lmp91200Vref::_5V => 0x02,
    };
    i2c_write_reg(bus, LMP91200_REF_VOLTAGE_REG, ref_voltage_val).await?;

    // TEMP_COMP_REG: Enable/disable temperature compensation
    let temp_comp_val = if lmp91200.temperature_compensation() {
        bit(0) // Enable temperature compensation
    } else {
        0x00
    };
    i2c_write_reg(bus, LMP91200_TEMP_COMP_REG, temp_comp_val).await?;

    // CALIBRATION_REG: Enable/disable calibration
    let cal_val = if lmp91200.calibration_enabled() {
        bit(0) // Enable calibration
    } else {
        0x00
    };
    i2c_write_reg(bus, LMP91200_CALIBRATION_REG, cal_val).await?;

    // Wait for configuration to take effect
    Timer::after_millis(100).await;

    Ok(())
}

async fn disable(bus: &impl SerialBus) -> Result<(), ChipError> {
    // Disable the sensor by setting config to sleep mode
    i2c_write_reg(bus, LMP91200_CONFIG_REG, 0x00).await?;
    Ok(())
}

pub async fn sample_lmp91200(lmp91200: &'static dyn Lmp91200Config, bus: &impl SerialBus, int_pin: &mut InterruptPin, channel: &'static Lmp91200Channel) -> Result<(), ChipError> {
    // Chip reset (if needed)
    i2c_write_reg(bus, LMP91200_CONFIG_REG, 0x00).await?;
    Timer::after_millis(5).await;

    // Confirm chip id
    let device_id = get_chip_id(bus).await?;
    info!("LMP91200 Device Id: 0x{:04X}", device_id);
    if device_id != LMP91200_DEVICE_ID {
        error!("LMP91200 chip id 0x{:04X} != 0x{:04X}", device_id, LMP91200_DEVICE_ID);
        Timer::after_millis(100).await;
        let device_id = get_chip_id(bus).await?;
        if device_id != LMP91200_DEVICE_ID {
            error!("Failed to communicate with LMP91200");
            return Err(ChipError::Logic);
        }
    }

    // Turn on continuous sampling
    enable(lmp91200, bus).await?;

    // Some constants for applying the correct timestamp
    let output_data_rate: i64 = match lmp91200.sample_rate() {
        Lmp91200Odr::_1Hz => 1,
        Lmp91200Odr::_2Hz => 2,
        Lmp91200Odr::_4Hz => 4,
        Lmp91200Odr::_8Hz => 8,
        Lmp91200Odr::_16Hz => 16,
        Lmp91200Odr::_32Hz => 32,
    };
    let per_sample_delay: i64 = 1_000_000 / output_data_rate;

    // As interrupts fire, read the data and hand off bytes
    let mut do_once = true;
    let mut last_sample_ts = utc_us_get();
    loop {
        if do_once {
            do_once = false;
        } else {
            int_pin.wait_for_low().await;
        }

        if STATE.load(Relaxed) != STATE_ACTIVE {
            info!("Stopping pH sampling");
            break;
        }

        // This is the nearest time to the interrupt that we can observe
        let utc_us = utc_us_get();
        let mut data = LmpMsg {
            first_utc_us: last_sample_ts + per_sample_delay,
            last_utc_us: utc_us,
            data: Vec::new(),
        };
        last_sample_ts = utc_us;

        // Check the status
        let status = i2c_read_reg(bus, LMP91200_STATUS_REG).await?;
        let data_ready = (status & bit(0)) != 0;
        
        if data_ready {
            // Read pH data (3 bytes: 2 bytes pH + 1 byte status)
            let mut ph_data = [0u8; 3];
            i2c_read_regs(bus, LMP91200_DATA_REG, &mut ph_data).await?;
            
            // Read temperature data (2 bytes)
            let mut temp_data = [0u8; 2];
            i2c_read_regs(bus, LMP91200_TEMP_REG, &mut temp_data).await?;
            
            // Combine all data
            let mut all_data = Vec::new();
            all_data.extend_from_slice(&ph_data);
            all_data.extend_from_slice(&temp_data);
            
            data.data = all_data;
        } else {
            // No data ready, send empty data
            data.data = Vec::new();
        }

        // Handoff this data
        if channel.try_send(data).is_err() {
            warn!("LMP91200 Channel full");
        }
    }

    // Turn off the chip before finishing the sampling
    disable(bus).await?;

    info!("Disabled LMP91200");

    Ok(())
}

/// Macro to create tasks for LMP91200 pH sensor
///
/// This macro generates a module with tasks and functions for managing a LMP91200 pH sensor instance.
/// It creates a channel for communication, async tasks for running the sensor, and a spawn function.
///
/// # Arguments
///
/// * `$inst` - The name of the instance (e.g., inst0, inst1)
/// * `$bus` - The bus type used for communication (e.g., TWIM0, TWIM1)
#[macro_export]
macro_rules! create_lmp91200_tasks {
    ($inst:ident, $bus:ty) => {
        pub mod $inst {
            use super::*;
            use core::mem::MaybeUninit;
            use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
            use embassy_sync::channel::Channel;
            use tsz_compress::prelude::*;

            use qap::app::serial_bus::SerialBus;
            use qap::drivers::LMP91200::PhRowCompressorImpl;
            use qap::drivers::LMP91200::Lmp91200Channel;
            use qap::*;

            /// Channel for communicating LMP91200 sensor data
            static CHANNEL: Lmp91200Channel = Channel::new();

            /// Static memory for the compressor, they use a lot of memory
            static mut COMPRESSOR: MaybeUninit<PhRowCompressorImpl> = MaybeUninit::uninit();

            static mut RAW_WORDS: [isize; qap::drivers::LMP91200::MAX_PH_WORDS] = [0isize; qap::drivers::LMP91200::MAX_PH_WORDS];

            /// Async task for running the LMP91200 sensor
            #[embassy_executor::task]
            async fn run(lmp91200: &'static dyn Lmp91200Config, bus: $bus, mut int_pin: InterruptPin) {
                qap::drivers::LMP91200::run_lmp91200(lmp91200, bus, int_pin, &CHANNEL).await;
            }

            /// Async task for running the LMP91200 sensor in blocking mode
            #[embassy_executor::task]
            async fn run_blocking(modality: u8, ble: &'static dyn BleConfig, lmp91200: &'static dyn Lmp91200Config) {
                unsafe {
                    COMPRESSOR.write(PhRowCompressorImpl::new(1));
                    let compressor = COMPRESSOR.assume_init_mut();
                    qap::drivers::LMP91200::run_blocking_lmp91200(modality, ble, lmp91200, &CHANNEL, compressor, &mut RAW_WORDS).await;
                }
            }

            /// Spawn function to start LMP91200 tasks
            pub fn spawn(modality: u8, spawners: &Spawners, ble: &'static dyn BleConfig, lmp91200: &'static dyn Lmp91200Config, bus: $bus, int_pin: InterruptPin) {
                spawners.spawn(NonBlocking, $inst::run(lmp91200, bus, int_pin));
                spawners.spawn(Blocking, $inst::run_blocking(modality, ble, lmp91200));
            }
        }
    };
}

pub type Lmp91200Channel = Channel<CriticalSectionRawMutex, LmpMsg, 8>;

pub async fn run_lmp91200(lmp91200: &'static dyn Lmp91200Config, bus: impl SerialBus, mut int_pin: InterruptPin, channel: &'static Lmp91200Channel) {
    let bus = &bus;
    loop {
        if STATE.load(Relaxed) != STATE_ACTIVE {
            Timer::after_millis(100).await;
            continue;
        }

        if channel
            .try_send(LmpMsg {
                first_utc_us: 0,
                last_utc_us: 0,
                data: Vec::new(),
            })
            .is_err()
        {
            warn!("LMP channel full");
        }

        match sample_lmp91200(lmp91200, bus, &mut int_pin, channel).await {
            Ok(_) => info!("LMP91200 sample success"),
            Err(e) => {
                warn!("LMP91200 sample failed: {:?}", e);
                Timer::after_secs(1).await;
            }
        }
    }
}

///
/// Process LMP91200 data
///
/// Parse chip data, compress, marshal, and send to BLE
///
pub async fn run_blocking_lmp91200(
    modality: u8, ble: &'static dyn BleConfig, lmp91200: &'static dyn Lmp91200Config, channel: &'static Lmp91200Channel, compressor: &'static mut PhRowCompressorImpl,
    raw_words: &'static mut [isize],
) {
    let rows_per_payload = lmp91200.rows_per_payload();
    let mut phase1 = UncompressedBuffer::new();
    let mut phase2 = CompressedBuffer::new();
    let mut ctx = Lmp91200CompressorContext {
        modality,
        l2cap_mtu: ble.l2cap_mtu(),
        ..Default::default()
    };

    let mut last_lmp = None;
    let mut lmp_cpu = Instant::now().elapsed();

    let hz = match lmp91200.sample_rate() {
        Lmp91200Odr::_1Hz => 1.0,
        Lmp91200Odr::_2Hz => 2.0,
        Lmp91200Odr::_4Hz => 4.0,
        Lmp91200Odr::_8Hz => 8.0,
        Lmp91200Odr::_16Hz => 16.0,
        Lmp91200Odr::_32Hz => 32.0,
    };

    // Convert raw ADC values to pH, temperature, and voltage
    let convert_to_ph = |adc_value: i16, ref_voltage: f32, gain: f32| -> f32 {
        let voltage = (adc_value as f32) * ref_voltage / (gain * 32768.0);
        // pH calculation based on Nernst equation
        // pH = 7.0 - (voltage - 0.0) / 0.0592
        let ph = 7.0 - (voltage - 0.0) / 0.0592;
        ph
    };

    let convert_to_temperature = |temp_adc: i16| -> f32 {
        // Temperature conversion based on LMP91200 datasheet
        // Temperature = (temp_adc * 0.0625) - 256
        (temp_adc as f32 * 0.0625) - 256.0
    };

    let mut first_ts = 0;

    loop {
        // Wait for the next data
        let lmp = channel.receive().await;

        if lmp.first_utc_us == 0 || lmp.last_utc_us == 0 {
            // Reset the processing
            first_ts = 0;
            last_lmp = None;
            lmp_cpu = Instant::now().elapsed();
            *compressor = PhRowCompressorImpl::new(rows_per_payload);
            continue;
        }

        // Process the data
        if let Some(LmpMsg { first_utc_us, last_utc_us, data }) = last_lmp {
            // timestamps are interpolated across the payload
            let elapsed = last_utc_us - first_utc_us;
            let words = data.len() / 5; // 3 bytes pH + 2 bytes temp per sample
            let tsd = (elapsed) as f64 / words as f64;

            // make sure the timestamps are valid
            if !((1577836800000000..2524608062367496).contains(&first_utc_us) && (1577836800000000..2524608062367496).contains(&last_utc_us)) || elapsed > 1_000_000 {
                last_lmp = Some(lmp);
                first_ts = 0;
                continue;
            }

            // the very first payload will use the first timestamp
            if first_ts == 0 {
                first_ts = first_utc_us;
            }

            let mut cpu_during_payload = Instant::now();

            // Parse the data to get pH, temperature, and voltage
            let mut idx: usize = 0;
            let mut words: usize = 0;
            let ref_voltage = lmp91200.reference_voltage().to_voltage();
            let gain = match lmp91200.gain() {
                Lmp91200Gain::_1 => 1.0,
                Lmp91200Gain::_2 => 2.0,
                Lmp91200Gain::_4 => 4.0,
                Lmp91200Gain::_8 => 8.0,
                Lmp91200Gain::_16 => 16.0,
                Lmp91200Gain::_32 => 32.0,
            };

            while idx < data.len() && words < MAX_PH_WORDS {
                if idx + 4 < data.len() {
                    // Parse pH data (2 bytes)
                    let ph_adc = i16::from_be_bytes([data[idx], data[idx + 1]]);
                    let ph_value = convert_to_ph(ph_adc, ref_voltage, gain);
                    
                    // Parse temperature data (2 bytes)
                    let temp_adc = i16::from_be_bytes([data[idx + 3], data[idx + 4]]);
                    let temp_value = convert_to_temperature(temp_adc);
                    
                    // Calculate voltage from pH ADC
                    let voltage = (ph_adc as f32) * ref_voltage / (gain * 32768.0);

                    unsafe {
                        *raw_words.get_unchecked_mut(words) = (ph_value * 1000.0) as isize; // pH * 1000 for precision
                        *raw_words.get_unchecked_mut(words + 1) = (temp_value * 100.0) as isize; // Temperature * 100 for precision
                        *raw_words.get_unchecked_mut(words + 2) = (voltage * 1000.0) as isize; // Voltage * 1000 for precision
                    };
                    words += 3;
                }
                idx += 5; // Move to next sample
            }
            let raw_words = &mut raw_words[..words];

            // Compress the data
            for i in 0..words/3 {
                let ph = (raw_words[i*3] / 1000) as i16;
                let temp = (raw_words[i*3 + 1] / 100) as i16;
                let voltage = (raw_words[i*3 + 2] / 1000) as i16;
                
                let row = PhRow { ph, temperature: temp, voltage };
                compressor.compress(row);
                let rows = compressor.row_count();
                if rows == rows_per_payload {
                    ctx.ts0 = first_ts;
                    ctx.ts1 = first_utc_us + (tsd * i as f64) as i64;
                    first_ts = ctx.ts1 + tsd as i64;
                    let payload_len = emit_payload(ble, compressor, &mut phase1, &mut phase2, &mut ctx);

                    lmp_cpu += cpu_during_payload.elapsed();
                    cpu_during_payload = Instant::now();
                    info!(
                        "LMP: {} us {} rows {} bytes {} B/row (pH: {}, temp: {}, voltage: {})",
                        lmp_cpu.as_micros(),
                        rows,
                        payload_len,
                        payload_len as f32 / rows as f32,
                        ph as f32 / 1000.0,
                        temp as f32 / 100.0,
                        voltage as f32 / 1000.0,
                    );

                    lmp_cpu = Instant::now().elapsed();
                }
            }

            lmp_cpu += cpu_during_payload.elapsed();
        }
        last_lmp = Some(lmp);
    }
}

#[derive(Default)]
struct Lmp91200CompressorContext {
    modality: u8,
    ts0: i64,
    ts1: i64,
    l2cap_mtu: usize,
}

impl CompressorContext for Lmp91200CompressorContext {
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
