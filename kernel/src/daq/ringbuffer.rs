//! Lock-Free SPSC (Single Producer Single Consumer) Ring Buffer
//!
//! Zero-copy pipeline for sensor data:
//!   Producer (ISR/driver) → writes raw samples into buffer
//!   Consumer (Silverfir WASM app) → reads samples directly from shared memory
//!
//! Design constraints:
//! - no_std, no locks, no syscalls on the hot path
//! - Power-of-2 capacity for branchless masking
//! - Atomic head/tail with Acquire/Release ordering (no SeqCst needed for SPSC)
//! - Cache-line padding to prevent false sharing between producer and consumer
//!
//! Memory layout (mappable directly into WASM linear memory):
//!   [0..4]     head (u32, written by producer)
//!   [64..68]   tail (u32, written by consumer)  ← separate cache line
//!   [128..]    data[] (f32 samples, capacity entries)

use core::sync::atomic::{AtomicU32, Ordering};

/// Cache line size on both ARM64 (Cortex-A76) and x86_64
const CACHE_LINE: usize = 64;

/// Default capacity: 8192 samples (32 KiB of f32 data)
/// Must be power of 2 for branchless masking
pub const DEFAULT_CAPACITY: usize = 8192;

/// Ring buffer header — cache-line padded to prevent false sharing
#[repr(C)]
pub struct RingHeader {
    /// Write position (only producer increments)
    pub head: AtomicU32,
    _pad_head: [u8; CACHE_LINE - 4],

    /// Read position (only consumer increments)
    pub tail: AtomicU32,
    _pad_tail: [u8; CACHE_LINE - 4],
}

/// SPSC Ring Buffer for f32 sensor samples
///
/// The buffer itself is a contiguous slice of f32 values.
/// Producer and consumer share the RingHeader for coordination.
pub struct SpscRingBuffer {
    header: &'static RingHeader,
    /// Raw pointer to data array — we use raw pointers to avoid UB
    /// when writing through &self (producer and consumer are different contexts)
    data_ptr: *mut f32,
    data_len: usize,
    mask: u32, // capacity - 1 (for branchless index wrapping)
}

impl SpscRingBuffer {
    /// Create a new ring buffer from a pre-allocated memory region.
    ///
    /// # Safety
    /// - `base` must point to at least `size_of::<RingHeader>() + capacity * 4` bytes
    /// - `base` must be aligned to at least 64 bytes (cache line)
    /// - `capacity` must be a power of 2
    /// - Memory must remain valid for the lifetime of the buffer
    pub unsafe fn from_raw(base: *mut u8, capacity: usize) -> Self {
        debug_assert!(capacity.is_power_of_two(), "capacity must be power of 2");
        debug_assert!(base as usize % CACHE_LINE == 0, "base must be cache-line aligned");

        let header = &*(base as *const RingHeader);

        // Data starts after the header (2 cache lines = 128 bytes)
        let data_ptr = base.add(core::mem::size_of::<RingHeader>()) as *mut f32;

        // Zero-initialize the header
        header.head.store(0, Ordering::Relaxed);
        header.tail.store(0, Ordering::Relaxed);

        SpscRingBuffer {
            header,
            data_ptr,
            data_len: capacity,
            mask: (capacity - 1) as u32,
        }
    }

    /// Total byte size needed for a ring buffer with given capacity
    pub const fn total_bytes(capacity: usize) -> usize {
        core::mem::size_of::<RingHeader>() + capacity * core::mem::size_of::<f32>()
    }

    // === Producer API (called from ISR / driver context) ===

    /// Try to write a batch of samples. Returns number of samples actually written.
    /// Never blocks — drops newest samples if buffer is full.
    pub fn produce(&self, samples: &[f32]) -> usize {
        let head = self.header.head.load(Ordering::Relaxed);
        let tail = self.header.tail.load(Ordering::Acquire);

        let available = self.mask.wrapping_add(1) - head.wrapping_sub(tail);
        let to_write = core::cmp::min(samples.len() as u32, available) as usize;

        for i in 0..to_write {
            let idx = (head.wrapping_add(i as u32) & self.mask) as usize;
            // Safety: idx is always < capacity due to mask
            unsafe {
                core::ptr::write_volatile(self.data_ptr.add(idx), samples[i]);
            }
        }

        // Ensure all writes are visible before advancing head
        #[cfg(target_arch = "aarch64")]
        crate::arch::aarch64::dmb_sy();
        #[cfg(target_arch = "x86_64")]
        core::sync::atomic::fence(Ordering::Release);

        self.header.head.store(head.wrapping_add(to_write as u32), Ordering::Release);
        to_write
    }

    /// Try to write a single sample. Returns true if successful.
    #[inline]
    pub fn produce_one(&self, sample: f32) -> bool {
        let head = self.header.head.load(Ordering::Relaxed);
        let tail = self.header.tail.load(Ordering::Acquire);

        if head.wrapping_sub(tail) > self.mask {
            return false; // Buffer full
        }

        let idx = (head & self.mask) as usize;
        unsafe {
            core::ptr::write_volatile(self.data_ptr.add(idx), sample);
        }

        self.header.head.store(head.wrapping_add(1), Ordering::Release);
        true
    }

    // === Consumer API (called from WASM runtime / Silverfir) ===

    /// Try to read up to `max` samples into `out`. Returns number of samples read.
    /// Never blocks — returns 0 if buffer is empty.
    pub fn consume(&self, out: &mut [f32]) -> usize {
        let tail = self.header.tail.load(Ordering::Relaxed);
        let head = self.header.head.load(Ordering::Acquire);

        let available = head.wrapping_sub(tail) as usize;
        let to_read = core::cmp::min(out.len(), available);

        for i in 0..to_read {
            let idx = (tail.wrapping_add(i as u32) & self.mask) as usize;
            unsafe {
                out[i] = core::ptr::read_volatile(self.data_ptr.add(idx));
            }
        }

        self.header.tail.store(tail.wrapping_add(to_read as u32), Ordering::Release);
        to_read
    }

    /// Number of samples currently available for reading
    #[inline]
    pub fn available(&self) -> u32 {
        let head = self.header.head.load(Ordering::Acquire);
        let tail = self.header.tail.load(Ordering::Relaxed);
        head.wrapping_sub(tail)
    }

    /// Number of free slots for writing
    #[inline]
    pub fn free_slots(&self) -> u32 {
        self.mask.wrapping_add(1) - self.available()
    }

    /// Whether the buffer is empty
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.available() == 0
    }

    /// Whether the buffer is full
    #[inline]
    pub fn is_full(&self) -> bool {
        self.available() > self.mask
    }

    /// Get the capacity of the buffer
    #[inline]
    pub fn capacity(&self) -> u32 {
        self.mask.wrapping_add(1)
    }

    /// Get a raw pointer to the data region (for WASM memory mapping)
    pub fn data_ptr(&self) -> *const f32 {
        self.data_ptr as *const f32
    }

    /// Get a raw pointer to the header (for WASM memory mapping)
    pub fn header_ptr(&self) -> *const RingHeader {
        self.header as *const RingHeader
    }
}

// === Multi-Channel Ring Buffer Manager ===

/// Manages one ring buffer per DAQ channel + one for the time domain.
/// All buffers are allocated from a single contiguous region for
/// easy mapping into WASM linear memory.
pub struct DaqRingManager {
    /// Ring buffer per channel (index 0 = time domain, 1..N = ADC channels)
    pub buffers: [Option<SpscRingBuffer>; 9], // 8 channels + 1 time
    pub base_ptr: *mut u8,
    pub total_size: usize,
}

impl DaqRingManager {
    /// Initialize all ring buffers from a contiguous memory region.
    ///
    /// # Safety
    /// - `base` must point to enough memory for `num_channels + 1` ring buffers
    /// - Memory must be cache-line aligned
    pub unsafe fn init(base: *mut u8, num_channels: usize, capacity_per_channel: usize) -> Self {
        let buf_size = SpscRingBuffer::total_bytes(capacity_per_channel);
        let total_count = num_channels + 1; // +1 for time domain
        let total_size = buf_size * total_count;

        // Zero-initialize entire region
        core::ptr::write_bytes(base, 0, total_size);

        const NONE: Option<SpscRingBuffer> = None;
        let mut buffers = [NONE; 9];

        for i in 0..total_count.min(9) {
            let offset = i * buf_size;
            buffers[i] = Some(SpscRingBuffer::from_raw(base.add(offset), capacity_per_channel));
        }

        DaqRingManager {
            buffers,
            base_ptr: base,
            total_size,
        }
    }

    /// Total bytes needed for all ring buffers
    pub const fn total_bytes(num_channels: usize, capacity_per_channel: usize) -> usize {
        SpscRingBuffer::total_bytes(capacity_per_channel) * (num_channels + 1)
    }

    /// Write samples for a specific channel (0 = time, 1-8 = ADC)
    pub fn produce_channel(&self, channel: usize, samples: &[f32]) -> usize {
        match &self.buffers[channel] {
            Some(buf) => buf.produce(samples),
            None => 0,
        }
    }

    /// Read samples from a specific channel
    pub fn consume_channel(&self, channel: usize, out: &mut [f32]) -> usize {
        match &self.buffers[channel] {
            Some(buf) => buf.consume(out),
            None => 0,
        }
    }

    /// Get available sample count across all channels (returns minimum)
    pub fn min_available(&self) -> u32 {
        let mut min = u32::MAX;
        for buf in &self.buffers {
            if let Some(b) = buf {
                let avail = b.available();
                if avail < min {
                    min = avail;
                }
            }
        }
        if min == u32::MAX { 0 } else { min }
    }
}
