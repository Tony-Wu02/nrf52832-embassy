// =======================================================================================
// File: xcg.rs
//
// Copyright (c) 2025 Northwestern University
// Authors:
// - Amey Kasbe
// - Jacob Trueb
// - Josh Prunty
// - Nathan Wang
//
// Licensed to Northwestern University. All Rights Reserved.
// =======================================================================================

//! # Wearable Electrocardiogram and Mechano-Acoustic Sensor :: nRF52840_XXAA
//!
//! This is firmware for a Mechano-acoustic sensor that has:
//! 1. LSM6DSO accelerometer
//! 2. MAX30001 ECG sensor
//! 3. V2S2000D PDM microphone
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

// Only allow --features=nand
#[cfg(all(feature = "no-nand", not(feature = "nand")))]
compiler_error!("NAND is present on this board");

ble_config! {
    name_prefix: "QXCG",
    firmware: "v0.2.4",
    conn_tx_power: 8,
    conn_interval_min: 12,
    conn_interval_max: 12,
    adv_interval: 200,
    adv_tx_power: nrf_softdevice::ble::TxPower::Minus8dBm,
}

lsm6dso_config! {
    sample_rate: Lsm6dsoOdr::_1667Hz,
    x_filt: (2, 10.0),
    y_filt: (2, 10.0),
    z_filt: (2, 100.0),
}

max30001_config! {
    sample_rate: Max30001Odr::_512Hz,
    gain: Max30001Gain::_40,
}

pdm_config! {
    pdm_freq: PdmClockFreq::_1032K,
    pdm_ratio: PdmDecimation::_64,
    pdm_gain: 20,
    z_decimate: (1, 8),
}

assign_resources! {
    adc: AdcResources {
        saadc: SAADC,
        ntc: P0_03,
        en: P0_02,
    }
    lsm6dso: ImuResources {
        spi: TWISPI0,
        sck: P0_06,
        miso: P0_08,
        mosi: P0_07,
        cs: P0_04,
        interrupt: P0_05,
    }
    max30001: Max30001Resources {
        spi: TWISPI1,
        sck: P0_11,
        miso: P1_09,
        mosi: P0_15,
        cs: P0_19,
        interrupt1: P0_27,
    }
    v2s2000d: V2sResources {
        pdm: PDM,
        clk: P0_31,
        din: P0_25,
    }
    nand: NandResources {
        spi: SPI2,
        sck: P0_28,
        miso: P0_21,
        mosi: P1_00,
        cs: P0_18,
        wp: P0_22,
        hold: P0_23,
    }
}

create_lsm6dso_tasks!(lsm6dso_inst0, SpimBus<'static, TWISPI0>);
create_max30001_tasks!(max30001_inst0, SpimBus<'static, TWISPI1>);
create_nand_tasks!(W25m02g<SpimBus<'static, SPI2>>);

extern crate alloc;

use drivers::w25m02g::W25m02g;
use embassy_nrf::pdm;
use embassy_nrf::pdm::Pdm;
use embassy_nrf::peripherals::{SPI2, TWISPI0, TWISPI1};
use embassy_nrf::saadc::ChannelConfig;
use embassy_nrf::saadc::Saadc;
use fixed::types::I7F1;
use qap::app::ble::*;
use qap::app::heap::init_heap;
use qap::app::heartbeat::utc_us_get;
use qap::app::pdm::*;
use qap::app::priority::SchedulePriority::{Blocking, NonBlocking};
use qap::app::priority::Spawners;
use qap::app::serial_bus::*;
use qap::app::state::*;
use qap::drivers::lsm6dso::*;
use qap::drivers::max30001::*;
use qap::utils::payload_packer::*;
use qap::*;

use assign_resources::assign_resources;
use core::sync::atomic::Ordering::Relaxed;
use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::wdt::{self, Watchdog};
use embassy_nrf::{bind_interrupts, interrupt, peripherals, saadc, spim};
use embassy_time::{Duration, Ticker};
use panic_probe as _;

use {defmt_rtt as _, nrf_softdevice as _};

bind_interrupts!(struct Irqs {
    SAADC => saadc::InterruptHandler;
    PDM => pdm::InterruptHandler<peripherals::PDM>;
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => spim::InterruptHandler<peripherals::TWISPI0>;
    SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1 => spim::InterruptHandler<peripherals::TWISPI1>;
    SPIM2_SPIS2_SPI2 => spim::InterruptHandler<peripherals::SPI2>;
});

mod app_batt {
    use super::*;
    use embassy_time::Timer;
    use qap::utils::cmp_ctx::*;
    use tsz_compress::prelude::*;

    mod batt {
        use tsz_compress::prelude::*;
        #[derive(Copy, Clone, CompressV2)]
        pub struct BattRow {
            #[tsz(delta = "i32")]
            pub(crate) ts: i64,
            pub(crate) mv: i16,
        }
    }
    use batt::compress::BattRowCompressorImpl;
    use batt::BattRow;

    struct BatteryCompressorContext;
    impl CompressorContext for BatteryCompressorContext {
        fn marshal_payload(&mut self, compressor: &mut impl tsz_compress::prelude::TszCompressV2) -> thin_vec::ThinVec<u8> {
            compressor.finish_thin()
        }

        fn op_code(&self) -> u8 {
            0x0
        }
    }

    #[embassy_executor::task]
    pub async fn run_cpu_battery_check(mut saadc: Saadc<1, 'static>, mut batt_mon_en: Output<'static>) {
        const ROWS_PER_PAYLOAD: usize = 10;
        let mut batt_compressor = BattRowCompressorImpl::new(ROWS_PER_PAYLOAD);
        let mut batt_ctx = BatteryCompressorContext;
        let mut phase1 = UncompressedBuffer::new();
        let mut phase2 = CompressedBuffer::new();
        let mut ticker = Ticker::every(Duration::from_secs(1));
        let mut idle_ticker = Ticker::every(Duration::from_secs(60));
        saadc.calibrate().await;
        loop {
            // Wait for the next tick
            ticker.next().await;

            // Wait for longer if not active
            if STATE.load(Relaxed) != STATE_ACTIVE {
                idle_ticker.next().await;
            }

            let ts = utc_us_get() / 1000;

            batt_mon_en.set_high();
            Timer::after_millis(10).await;

            // saadc.calibrate().await;
            let mut adc_val = [0; 1];
            saadc.sample(&mut adc_val).await;
            batt_mon_en.set_low();

            // adc = (vp - vn) * (gain / ref) * 2^(resolution - m)
            // vp = adc / (gain / ref * 2^(resolution - m))
            let to_uv = |adc: i16| -> i32 { ((adc as f32) / (((1.0 / 6.0) / (0.6 * 1e6)) * (1 << 14) as f32)) as i32 };
            let uv = to_uv(adc_val[0]);
            let batt_mv = (28_000 + ((uv / 100 - 3_581) * (42_000 - 28_000) / (5_372 - 3_581))) / 10;

            // Send the battery level over BLE
            if BATT_NOTIF.load(Relaxed) || EVENT_NOTIF.load(Relaxed) {
                // Block until BLE is ready, especially durig EXPORT
                BLE_CHANNEL.send(BleMessage::BatteryLevel(batt_mv)).await
            }

            if batt_mv > 200 && batt_mv < 3600 {
                BATTERY_LOW.store(true, Relaxed);
                if STATE.load(Relaxed) != STATE_LOW_BATT {
                    STATE_CHANGE_CHANNEL.send(STATE_LOW_BATT).await;
                }
            } else if batt_mv > 3800 {
                BATTERY_LOW.store(false, Relaxed);
                if STATE.load(Relaxed) == STATE_LOW_BATT {
                    STATE_CHANGE_CHANNEL.send(STATE_IDLE).await;
                }
            }

            if ts == 0 {
                continue;
            }

            let row = BattRow { ts, mv: batt_mv as i16 };
            batt_compressor.compress(row);
            if is_conn_stable() || batt_compressor.row_count() >= ROWS_PER_PAYLOAD {
                let _ = emit_payload(&BLE_CONF, &mut batt_compressor, &mut phase1, &mut phase2, &mut batt_ctx);
            }
        }
    }
}

static PDM_CHANNEL: PdmChannel = PdmChannel::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    // Configure for low-power SoftDevice operation
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    config.lfclk_source = embassy_nrf::config::LfclkSource::ExternalXtal;
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.dcdc.reg1 = true;
    let p = embassy_nrf::init(config);
    let mut r = split_resources!(p);

    // Confirm to the bootloader that we are alive by consuming the watchdog
    let wdt_config = wdt::Config::try_new(&p.WDT).unwrap();
    let (_wdt, [wdt_handle]) = match Watchdog::try_new(p.WDT, wdt_config) {
        Ok(x) => x,
        Err(_) => {
            // Watchdog already active with the wrong number of handles, waiting for it to timeout...
            loop {
                cortex_m::asm::wfe();
            }
        }
    };

    // Okay, we are booted.
    info!("Hello World!");

    // Prepare to handle allocations
    unsafe {
        init_heap();
    }

    // Initialize the application state as IDLE
    STATE.store(STATE_IDLE, Relaxed);

    info!("Initialized peripherals");

    interrupt::SAADC.set_priority(Priority::P3);
    interrupt::PDM.set_priority(Priority::P3);
    interrupt::SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0.set_priority(Priority::P3);
    interrupt::SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1.set_priority(Priority::P3);
    interrupt::SPIM2_SPIS2_SPI2.set_priority(Priority::P3);

    // Configure SAADC to handoff to task
    let mut config = embassy_nrf::saadc::Config::default();
    config.resolution = embassy_nrf::saadc::Resolution::_14BIT;
    config.oversample = embassy_nrf::saadc::Oversample::OVER16X;
    let channel_0_config = ChannelConfig::single_ended(&mut r.adc.ntc);
    let saadc = Saadc::new(r.adc.saadc, Irqs, config, [channel_0_config]);
    interrupt::SAADC.set_priority(Priority::P3);

    // Configure PDM for microphone
    let config = embassy_nrf::pdm::Config {
        frequency: PDM_CONF.pdm_freq().into(),
        operation_mode: embassy_nrf::pdm::OperationMode::Mono,
        ratio: PDM_CONF.pdm_decimation().into(),
        gain_left: PDM_CONF.pdm_gain(),
        gain_right: PDM_CONF.pdm_gain(),
        ..Default::default()
    };
    let pdm = Pdm::new(r.v2s2000d.pdm, Irqs, r.v2s2000d.clk, r.v2s2000d.din, config);

    // Configure SPIM for IMU
    let mut config = embassy_nrf::spim::Config::default();
    config.frequency = embassy_nrf::spim::Frequency::M8;
    let lsm6dso_spim = embassy_nrf::spim::Spim::new(r.lsm6dso.spi, Irqs, r.lsm6dso.sck, r.lsm6dso.miso, r.lsm6dso.mosi, config);
    let lsm6dso_spi_cs = Output::new(r.lsm6dso.cs, Level::High, OutputDrive::Standard); // active low
    let lsm6dso_spi_int = Input::new(r.lsm6dso.interrupt, Pull::None); // active high
    let lsm6dso_bus = SpimBus::new(lsm6dso_spim, lsm6dso_spi_cs);

    // Configure SPIM for MAX30001
    let mut config = embassy_nrf::spim::Config::default();
    config.frequency = embassy_nrf::spim::Frequency::M8;
    let max30001_spim = embassy_nrf::spim::Spim::new(r.max30001.spi, Irqs, r.max30001.sck, r.max30001.miso, r.max30001.mosi, config);
    let max30001_spi_cs = Output::new(r.max30001.cs, Level::High, OutputDrive::Standard); // active low
    let max30001_spi_int1 = Input::new(r.max30001.interrupt1, Pull::None); // active low
    let max30001_bus = SpimBus::new(max30001_spim, max30001_spi_cs);

    // Create spawners for each of the application task priorities
    let spawners = Spawners::new(spawner);
    spawners.spawn_normal_tasks(wdt_handle, &BLE_CONF);

    let batt_mon_en = Output::new(r.adc.en, Level::Low, OutputDrive::Standard); // active high
    spawners.spawn(Blocking, app_batt::run_cpu_battery_check(saadc, batt_mon_en));

    spawners.spawn(NonBlocking, qap::app::pdm::run_pdm(&PDM_CONF, pdm, &PDM_CHANNEL));
    spawners.spawn(Blocking, qap::app::pdm::run_cpu_pdm_mono(50, &BLE_CONF, &PDM_CONF, &PDM_CHANNEL));

    lsm6dso_inst0::spawn(48, &spawners, &BLE_CONF, &LSM6DSO_CONF, lsm6dso_bus, lsm6dso_spi_int);
    max30001_inst0::spawn(51, &spawners, &BLE_CONF, &MAX30001_CONF, max30001_bus, max30001_spi_int1);
    // Spawn NAND tasks
    let mut config = embassy_nrf::spim::Config::default();
    config.frequency = embassy_nrf::spim::Frequency::M8;
    let nand_spim = embassy_nrf::spim::Spim::new(r.nand.spi, Irqs, r.nand.sck, r.nand.miso, r.nand.mosi, config);
    let nand_spi_cs = Output::new(r.nand.cs, Level::High, OutputDrive::Standard); // active low
    let nand_wp = Output::new(r.nand.wp, Level::Low, OutputDrive::Standard); // active low
    let nand_hold = Output::new(r.nand.hold, Level::Low, OutputDrive::Standard); // active low
    let nand_bus = SpimBus::new(nand_spim, nand_spi_cs);
    interrupt::SPIM2_SPIS2_SPI2.set_priority(Priority::P3);
    match W25m02g::new(nand_wp, nand_hold, nand_bus).await {
        Ok(w25m02g) => nand::spawn(&spawners, &BLE_CONF, w25m02g, false),
        Err(e) => error!("Failed to initialize NAND: {}", e),
    }

    // Handle application transitions between streaming and exporting in async blocking fashion
    loop {
        let _ = handle_state_change(nop_setup, nop_teardown).await;
    }
}
