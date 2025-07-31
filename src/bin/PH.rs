// =======================================================================================
// File: PH.rs
//
// Copyright (c) 2025 Vanderbilt University
// Authors:
// - Tengyue Wu
//
// Licensed to Vanderbilt University. All Rights Reserved.
// =======================================================================================

//! # pH Sensor :: nRF52832_XXAA
//!
//! This is firmware for a pH sensor that has:
//! 1. LMP91200 pH sensor
//! 2. CAT24C32 EEPROM storage
//! 3. Battery monitoring
//!

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

// Only allow --features=cat
#[cfg(all(feature = "no-cat", not(feature = "cat")))]
compiler_error!("CAT is present on this board");

ble_config! {
    name_prefix: "PHSensor",
    firmware: "v1.0.0",
    conn_tx_power: 8,
    conn_interval_min: 12,
    conn_interval_max: 12,
    adv_interval: 200,
    adv_tx_power: nrf_softdevice::ble::TxPower::Minus8dBm,
}

lmp91200_config! {
    sample_rate: Lmp91200Odr::_4Hz,
    gain: Lmp91200Gain::_8,
    reference_voltage: Lmp91200Vref::_3_3V,
    temperature_compensation: true,
    calibration_enabled: true,
    fifo_threshold: 16,
    rows_per_payload: 50,
}

// Resource definitions
struct AdcResources {
    saadc: peripherals::SAADC,
    nt_pin: peripherals::P0_29,
    en_pin: peripherals::P0_30,
}

struct Lmp91200Resources {
    i2c: peripherals::TWIM0,
    sda: peripherals::P0_26,
    scl: peripherals::P0_27,
    interrupt: peripherals::P0_28,
}

struct CatResources {
    i2c: peripherals::TWIM1,
    sda: peripherals::P0_31,
    scl: peripherals::P0_30,
}

create_lmp91200_tasks!(lmp0, I2cBus<'static, TWIM0>);

extern crate alloc;

use embassy_nrf::peripherals::{TWIM0, TWIM1, SAADC};
use embassy_nrf::saadc::ChannelConfig;
use embassy_nrf::saadc::Saadc;
use qap::app::ble::*;
use qap::app::heap::init_heap;
use qap::app::heartbeat::utc_us_get;
use qap::app::priority::SchedulePriority::{Blocking, NonBlocking};
use qap::app::priority::Spawners;
use qap::app::serial_bus::*;
use qap::app::state::*;
use qap::app::driver::LMP91200::*;
use qap::utils::payload_packer::*;
use qap::*;

// assign_resources macro is not available, using manual resource allocation
use core::sync::atomic::Ordering::Relaxed;
use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::wdt::{self, Watchdog};
use embassy_nrf::{bind_interrupts, interrupt, peripherals, saadc, twim};
use embassy_time::{Duration, Ticker};
use panic_probe as _;

use {defmt_rtt as _, nrf_softdevice as _};

bind_interrupts!(struct Irqs {
    SAADC => saadc::InterruptHandler;
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => twim::InterruptHandler<peripherals::TWIM0>;
    SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1 => twim::InterruptHandler<peripherals::TWIM1>;
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

            let mut adc_val = [0; 1];
            saadc.sample(&mut adc_val).await;
            batt_mon_en.set_low();

            // Convert ADC value to battery voltage
            let to_uv = |adc: i16| -> i32 { ((adc as f32) / (((1.0 / 6.0) / (0.6 * 1e6)) * (1 << 14) as f32)) as i32 };
            let uv = to_uv(adc_val[0]);
            let batt_mv = (28_000 + ((uv / 100 - 3_581) * (42_000 - 28_000) / (5_372 - 3_581))) / 10;

            // Send the battery level over BLE
            if BATT_NOTIF.load(Relaxed) || EVENT_NOTIF.load(Relaxed) {
                // Block until BLE is ready, especially during EXPORT
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
    
    // Manual resource allocation
    let adc_resources = AdcResources {
        saadc: p.SAADC,
        nt_pin: p.P0_29,
        en_pin: p.P0_30,
    };
    let lmp91200_resources = Lmp91200Resources {
        i2c: p.TWIM0,
        sda: p.P0_26,
        scl: p.P0_27,
        interrupt: p.P0_28,
    };
    let cat_resources = CatResources {
        i2c: p.TWIM1,
        sda: p.P0_31,
        scl: p.P0_30,
    };

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
    info!("pH Sensor Starting...");

    // Prepare to handle allocations
    unsafe {
        init_heap();
    }

    // Initialize the application state as IDLE
    STATE.store(STATE_IDLE, Relaxed);

    info!("Initialized peripherals");

    interrupt::SAADC.set_priority(Priority::P3);
    interrupt::SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0.set_priority(Priority::P3);
    interrupt::SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1.set_priority(Priority::P3);

    // Configure SAADC for battery monitoring
    let mut config = embassy_nrf::saadc::Config::default();
    config.resolution = embassy_nrf::saadc::Resolution::_14BIT;
    config.oversample = embassy_nrf::saadc::Oversample::OVER16X;
    let channel_0_config = ChannelConfig::single_ended(&mut adc_resources.nt_pin);
    let saadc = Saadc::new(adc_resources.saadc, Irqs, config, [channel_0_config]);
    interrupt::SAADC.set_priority(Priority::P3);

    // Configure I2C for LMP91200
    let lmp91200_i2c = embassy_nrf::twim::Twim::new(lmp91200_resources.i2c, Irqs, lmp91200_resources.sda, lmp91200_resources.scl, Default::default());
    let lmp91200_bus = I2cBus::new(lmp91200_i2c);
    let lmp91200_int = Input::new(lmp91200_resources.interrupt, Pull::None); // active high
    let lmp91200_int = InterruptPin::new(lmp91200_int);

    // Configure I2C for CAT24C32 (if enabled)
    #[cfg(feature = "cat")]
    let cat_i2c = embassy_nrf::twim::Twim::new(cat_resources.i2c, Irqs, cat_resources.sda, cat_resources.scl, Default::default());

    // Create spawners for each of the application task priorities
    let spawners = Spawners::new(spawner);
    spawners.spawn_normal_tasks(wdt_handle, &BLE_CONF);

    let batt_mon_en = Output::new(adc_resources.en_pin, Level::Low, OutputDrive::Standard); // active high
    spawners.spawn(Blocking, app_batt::run_cpu_battery_check(saadc, batt_mon_en));

    // Spawn LMP91200 pH sensor tasks
    lmp0::spawn(0x01, &spawners, &LMP91200_CONF, &LMP91200_CONF, lmp91200_bus, lmp91200_int);

    // Spawn CAT storage tasks (if enabled)
    #[cfg(feature = "cat")]
    {
        let cat_bus = I2cBus::new(cat_i2c);
        spawners.spawn(Blocking, qap::app::cat::run_cat());
    }

    // Handle application transitions between streaming and exporting in async blocking fashion
    loop {
        let _ = handle_state_change(nop_setup, nop_teardown).await;
    }
}

