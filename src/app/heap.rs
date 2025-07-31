// =======================================================================================
// File: heap.rs
//
// Copyright (c) 2025 Northwestern University
// Authors:
// - Amey Kasbe
// - Jacob Trueb
// - Josh Prunty
//
// Licensed to Northwestern University. All Rights Reserved.
// =======================================================================================

use core::sync::atomic::AtomicIsize;
use embedded_alloc::TlsfHeap as Heap;

/// The total size of the heap that is available for dynamic memory allocation
pub const HEAP_SIZE: usize = 189 * 1024;

/// A couple large allocations cause significant fragmentation
#[cfg(feature = "cat")]
pub const HEAP_SIZE_FOR_SMALL_ALLOCS: usize = HEAP_SIZE - (1 << 14);
#[cfg(not(feature = "cat"))]
pub const HEAP_SIZE_FOR_SMALL_ALLOCS: usize = HEAP_SIZE;

/// Assume a fragmentation rate of 15% on the usable heap
pub const USABLE_HEAP_SIZE: usize = HEAP_SIZE_FOR_SMALL_ALLOCS * 85 / 100;
#[cfg(feature = "cat")]
// Payloads hold wire and ram format held in memory
pub const MAX_HEAP_USAGE_FOR_PAYLOADS: usize = USABLE_HEAP_SIZE / 8;
#[cfg(not(feature = "cat"))]
pub const MAX_HEAP_USAGE_FOR_PAYLOADS: usize = USABLE_HEAP_SIZE * 3 / 4;

#[repr(align(1024))]
pub struct HeapBuffer([u8; HEAP_SIZE]);
pub static mut HEAP_BUFFER: HeapBuffer = HeapBuffer([0; HEAP_SIZE]);
#[global_allocator]
pub static HEAP: Heap = Heap::empty();
pub static PAYLOAD_SIZE: AtomicIsize = AtomicIsize::new(0);

/// Initialize the global heap
///
/// # Safety
/// This function must only be called once and before any other use of a function that may allocate
///
pub unsafe fn init_heap() {
    HEAP.init(HEAP_BUFFER.0.as_mut_ptr() as usize, HEAP_BUFFER.0.len());
}
