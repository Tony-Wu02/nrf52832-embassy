// =======================================================================================
// File: priority.rs
//
// Copyright (c) 2025 Northwestern University
// Authors:
// - Amey Kasbe
// - Jacob Trueb
// - Josh Prunty
//
// Licensed to Northwestern University. All Rights Reserved.
// =======================================================================================

use defmt::unwrap;
use embassy_executor::{InterruptExecutor, SendSpawner, SpawnToken, Spawner};
use embassy_nrf::interrupt;
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::wdt::WatchdogHandle;

use super::ble::BleConfig;

// Priority 5 Executor
static EXECUTOR_SWI0: InterruptExecutor = InterruptExecutor::new();
#[interrupt]
unsafe fn SWI0_EGU0() {
    EXECUTOR_SWI0.on_interrupt()
}

// Priority 6 Executor
static EXECUTOR_SWI3: InterruptExecutor = InterruptExecutor::new();
#[interrupt]
unsafe fn SWI3_EGU3() {
    EXECUTOR_SWI3.on_interrupt()
}

/// A schedule priority is a description of how a task will be placed on an executor
pub enum SchedulePriority {
    /// Blocking - A task that blocks the CPU and needs to run on a preemptive main thread
    Blocking,
    /// NonBlocking - A task that may block the CPU for a very short time (<100µs)
    NonBlocking,
    /// TimeSensitive - A task that must use minimal CPU time.
    /// Time sensitive tasks that run for too long may disrupt system behavior
    TimeSensitive,
}

/// Async tasks can be spawned on the appropriate priority executor based on spawners started from the main executor.
pub struct Spawners {
    swi_high: SendSpawner,
    swi_low: SendSpawner,
    preempt: Spawner,
}

impl Spawners {
    /// Create an instance of all of the spawners needed for the application
    ///
    /// # SAFETY
    /// This function must be called once. We cannot restart executors.
    pub fn new(preempt: Spawner) -> Self {
        // The last priority for the SoftDevice is P4.
        // We can use P5 and P6 for our application tasks.
        interrupt::SWI0_EGU0.set_priority(Priority::P5);
        interrupt::SWI3_EGU3.set_priority(Priority::P6);
        // The SoftDevice is using the other software interrupt event generator units.
        let swi_high = EXECUTOR_SWI0.start(interrupt::SWI0_EGU0);
        let swi_low = EXECUTOR_SWI3.start(interrupt::SWI3_EGU3);
        Self { swi_high, swi_low, preempt }
    }

    /// Spawn a task on the appropriate executor based on the priority
    ///
    /// # Note
    /// This function will panic if the task cannot be spawned.
    /// Make sure to allocate enough tasks with the embassy task macro.
    /// The default pool size for `#[embassy_executor::task]` is 1.
    pub fn spawn<S>(&self, priority: SchedulePriority, task_token: SpawnToken<S>)
    where
        S: Send + 'static,
    {
        let rslt = match priority {
            SchedulePriority::Blocking => self.preempt.spawn(task_token),
            SchedulePriority::NonBlocking => self.swi_low.spawn(task_token),
            SchedulePriority::TimeSensitive => self.swi_high.spawn(task_token),
        };
        unwrap!(rslt);
    }

    pub fn spawn_normal_tasks(&self, wdt_handle: WatchdogHandle, ble_conf: &'static dyn BleConfig) {
        use SchedulePriority::*;
        // Spawn time-sensitive tasks that could crash the system
        self.spawn(TimeSensitive, crate::app::watchdog::run_watchdog(wdt_handle));
        self.spawn(TimeSensitive, crate::app::ble::run_ble(ble_conf));
        #[cfg(not(feature = "no-soc-die-temp"))]
        {
            self.spawn(TimeSensitive, crate::app::die_temp::run_die_temp());
        }

        // Spawn non-blocking tasks that manage hardware peripherals directly
        self.spawn(NonBlocking, crate::app::ble::run_deferred_ble());

        // Run low priority, expensive/blocking work in thread mode
        #[cfg(not(feature = "no-soc-die-temp"))]
        {
            self.spawn(Blocking, crate::app::die_temp::run_cpu_die_temp(ble_conf));
        }
        self.spawn(Blocking, crate::app::heartbeat::run_cpu_heartbeat(ble_conf));
        self.spawn(Blocking, crate::utils::dfu::run_cpu_dfu());
    }
}
