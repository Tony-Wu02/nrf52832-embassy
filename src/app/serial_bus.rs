// =======================================================================================
// File: serial_bus.rs
//
// Copyright (c) 2025 Northwestern University
// Authors:
// - Jacob Trueb
//
// Licensed to Northwestern University. All Rights Reserved.
// =======================================================================================

use core::cell::UnsafeCell;

use defmt::Format;
use embassy_nrf::gpio::Output;
use embassy_nrf::spim::{self, Error as SpimError, Spim};
use embassy_nrf::twim::{self, Error as TwimError, Twim};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

use crate::ChipSelectPin;

/// Error type for serial buses.
/// Encompasses errors from both I2C (TWIM) and SPI (SPIM) operations on the NRF52840.
#[derive(Debug, Copy, Clone, Format, PartialEq, Eq)]
pub enum SerialBusError {
    /// TWIM (I2C) Error
    Twim(TwimError),
    /// SPIM (SPI) Error
    Spim(SpimError),
    /// Chip Part Id Error
    PartId,
}

impl From<TwimError> for SerialBusError {
    fn from(error: TwimError) -> Self {
        SerialBusError::Twim(error)
    }
}

impl From<SpimError> for SerialBusError {
    fn from(error: SpimError) -> Self {
        SerialBusError::Spim(error)
    }
}

/// Trait representing a generic serial bus interface.
///
/// This trait can be implemented for various serial communication protocols
/// such as SPI (SPIM0, SPIM1, SPIM2, SPIM3) or I2C (TWI0, TWI1).
///
/// Implementations of this trait handle communication with slave devices and may
/// manage bus sharing between multiple integrated circuits.
///
/// If synchronization is required, it should be handled by the implementor of this trait.
#[allow(async_fn_in_trait)]
pub trait SerialBus: Sized {
    /// Performs a bidirectional data transfer over the serial bus.
    ///
    /// # Arguments
    /// * `tx` - Slice of bytes to transmit
    /// * `rx` - Mutable slice to receive bytes into
    ///
    /// # Returns
    /// `Result<(), SerialBusError>` indicating success or the type of error encountered
    ///
    /// # Note
    /// The number of bytes received will match the length of the `tx` slice.
    async fn xfer(&self, tx: &[u8], rx: &mut [u8]) -> Result<(), SerialBusError>;
}

/* Bus Types */

/// Represents an uncontested SPI bus.
pub struct SpimBus<'b, T: spim::Instance> {
    state: UnsafeCell<SpimBusState<'b, T>>,
}

/// Represents an uncontested I2C bus.
pub struct TwimBus<'b, T: twim::Instance> {
    state: UnsafeCell<TwimBusState<'b, T>>,
}

/// Represents a reference to a shared SPI bus for a specific slave device.
///
/// This type implements `SerialBus` and is created from a `SharedSpimBus`.
pub struct SpimBusRef<'b, T: spim::Instance> {
    state: &'b Shared<Spim<'b, T>>,
    cs: UnsafeCell<ChipSelectPin>,
}

/// Represents a reference to a shared I2C bus for a specific slave device.
///
/// This type implements `SerialBus` and is created from a `SharedTwimBus`.
pub struct TwimBusRef<'b, T: twim::Instance> {
    state: &'b Shared<Twim<'b, T>>,
    addr: u8,
}

/// Represents a shared SPI bus that can be used by multiple slave devices.
///
/// This type does not implement `SerialBus` directly. Instead, use `select_chip`
/// to create a `SpimBusRef` for a specific slave device.
pub struct SharedSpimBus<'b, T: spim::Instance> {
    state: Shared<Spim<'b, T>>,
}

/// Represents an SPI bus with multiple devices that cannot be shared across tasks
///
/// It is considered unsafe because the internals use pointers that will violate
/// mutable aliasing requirements if used concurrently. Therefore, it is only safe
/// to use within the same task.
///
/// This type does not implement `SerialBus` directly. Instead, use `select_chip`
/// to create a `LoadedSpimBusRef`
pub struct UnsafeSharedSpimBus<'b, const N: usize, T: spim::Instance> {
    spim: *mut Spim<'b, T>,
    cs: [Output<'b>; N],
}

pub struct UnsafeSharedSpimBusRef<'b, T: spim::Instance> {
    spim: *mut Spim<'b, T>,
    cs: *mut Output<'b>,
}

unsafe impl<'b, T: spim::Instance> Send for UnsafeSharedSpimBusRef<'b, T> {}
unsafe impl<'b, T: spim::Instance> Sync for UnsafeSharedSpimBusRef<'b, T> {}

/// Represents a shared I2C bus that can be used by multiple slave devices.
///
/// This type does not implement `SerialBus` directly. Instead, use `select_chip`
/// to create a `TwimBusRef` for a specific slave device.
pub struct SharedTwimBus<'b, T: twim::Instance> {
    state: Shared<Twim<'b, T>>,
}

/// Type alias for a shared bus protected by an async mutex.
type Shared<T> = Mutex<CriticalSectionRawMutex, T>;

/* Bus State and Implementation */

impl<'b, T: spim::Instance> SerialBus for SpimBus<'b, T> {
    async fn xfer(&self, tx: &[u8], rx: &mut [u8]) -> Result<(), SerialBusError> {
        let state = unsafe { &mut *self.state.get() };
        state.cs.set_low();
        cortex_m::asm::delay(5);
        state.spim.transfer(rx, tx).await?;
        state.cs.set_high();
        Ok(())
    }
}

impl<'b, T: spim::Instance> SerialBus for UnsafeSharedSpimBusRef<'b, T> {
    async fn xfer(&self, tx: &[u8], rx: &mut [u8]) -> Result<(), SerialBusError> {
        let spim = unsafe { &mut *self.spim };
        let cs = unsafe { &mut *self.cs };
        cs.set_low();
        cortex_m::asm::delay(5);
        spim.transfer(rx, tx).await?;
        cs.set_high();
        Ok(())
    }
}

impl<'b, T: twim::Instance> SerialBus for TwimBus<'b, T> {
    async fn xfer(&self, tx: &[u8], rx: &mut [u8]) -> Result<(), SerialBusError> {
        let state = unsafe { &mut *self.state.get() };
        if rx.is_empty() {
            state.twim.write(state.addr, tx).await?;
        } else {
            state.twim.write_read(state.addr, tx, rx).await?;
        }
        Ok(())
    }
}

impl<'b, T: spim::Instance> SerialBus for SpimBusRef<'b, T> {
    async fn xfer(&self, tx: &[u8], rx: &mut [u8]) -> Result<(), SerialBusError> {
        let mut spim = self.state.lock().await;
        let cs = unsafe { &mut *self.cs.get() };
        cs.set_low();
        cortex_m::asm::delay(5);
        spim.transfer(rx, tx).await?;
        cs.set_high();
        Ok(())
    }
}

impl<'b, T: twim::Instance> SerialBus for TwimBusRef<'b, T> {
    async fn xfer(&self, tx: &[u8], rx: &mut [u8]) -> Result<(), SerialBusError> {
        let mut twim = self.state.lock().await;
        if rx.is_empty() {
            twim.write(self.addr, tx).await?;
        } else {
            twim.write_read(self.addr, tx, rx).await?;
        }
        Ok(())
    }
}

struct SpimBusState<'b, T: spim::Instance> {
    spim: Spim<'b, T>,
    cs: ChipSelectPin,
}

impl<'b, T: spim::Instance> SpimBus<'b, T> {
    /// Creates a new uncontested SPI bus.
    pub fn new(spim: Spim<'b, T>, cs: ChipSelectPin) -> Self {
        Self {
            state: UnsafeCell::new(SpimBusState { spim, cs }),
        }
    }
}

struct TwimBusState<'b, T: twim::Instance> {
    twim: Twim<'b, T>,
    addr: u8,
}

impl<'b, T: twim::Instance> TwimBus<'b, T> {
    /// Creates a new uncontested I2C bus.
    pub fn new(twim: Twim<'b, T>, addr: u8) -> Self {
        Self {
            state: UnsafeCell::new(TwimBusState { twim, addr }),
        }
    }
}

impl<'b, T: spim::Instance> SharedSpimBus<'b, T> {
    /// Creates a new shared SPI bus.
    pub fn new(spim: Spim<'b, T>) -> Self {
        Self { state: Mutex::new(spim) }
    }

    /// Creates a `SpimBusRef` for a specific slave device on this shared SPI bus.
    ///
    /// # Safety
    /// The caller must ensure that the `SharedSpimBus` is never dropped after
    /// the `SpimBusRef` has been created.
    pub unsafe fn select_chip(&self, cs: ChipSelectPin) -> SpimBusRef<'static, T> {
        #[allow(clippy::unnecessary_cast)]
        let ptr = self as *const SharedSpimBus<'b, T> as *const SharedSpimBus<'static, T>;
        let this = &*ptr;
        SpimBusRef {
            state: &this.state,
            cs: UnsafeCell::new(cs),
        }
    }
}

impl<'b, const N: usize, T: spim::Instance> UnsafeSharedSpimBus<'b, N, T> {
    pub fn new(spim: &mut Spim<'b, T>, chip_selects: [Output<'b>; N]) -> Self {
        Self { spim, cs: chip_selects }
    }

    /// Creates a bus reference by copying pointers
    ///
    /// # Safety
    /// Any refs selected from this bus must not be aliased at the same time!
    pub unsafe fn select_chip(&mut self, idx: usize) -> UnsafeSharedSpimBusRef<'b, T> {
        UnsafeSharedSpimBusRef {
            spim: self.spim,
            cs: &mut self.cs[idx],
        }
    }
}

impl<'b, T: twim::Instance> SharedTwimBus<'b, T> {
    /// Creates a new shared I2C bus.
    pub fn new(twim: Twim<'b, T>) -> Self {
        Self { state: Mutex::new(twim) }
    }

    /// Creates a `TwimBusRef` for a specific slave device on this shared I2C bus.
    ///
    /// # Safety
    /// The caller must ensure that the `SharedTwimBus` is never dropped after
    /// the `TwimBusRef` has been created.
    pub unsafe fn select_chip(&self, addr: u8) -> TwimBusRef<'static, T> {
        #[allow(clippy::unnecessary_cast)]
        let ptr = self as *const SharedTwimBus<'b, T> as *const SharedTwimBus<'static, T>;
        let this = &*ptr;
        TwimBusRef { state: &this.state, addr }
    }
}
