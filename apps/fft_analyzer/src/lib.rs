//! FFT Spectrum Analyzer — Radix-2 DIT for Folkering DAQ
//!
//! Performs a 256-point FFT on incoming ADC data to produce a
//! frequency spectrum. The magnitude of each bin is written to
//! the output ring buffer.
//!
//! The FFT uses the Cooley-Tukey radix-2 decimation-in-time algorithm
//! implemented entirely in fixed-point-friendly f32 arithmetic.
//!
//! Output: 128 magnitude bins (DC to Nyquist) per channel per frame.
//! Bin width = sample_rate / FFT_SIZE
//!
//! Example: at 20 kHz sample rate, each bin covers ~78 Hz.

#![no_std]

extern "C" {
    fn folk_daq_read_samples(channel: i32, buf_ptr: i32, max_samples: i32) -> i32;
    fn folk_daq_available() -> i32;
    fn folk_daq_sample_rate() -> i32;
    fn folk_daq_channel_count() -> i32;
    fn folk_daq_write_output(channel: i32, buf_ptr: i32, num_samples: i32) -> i32;
    fn folk_log(ptr: i32, len: i32) -> i32;
}

/// FFT size — must be power of 2
const FFT_SIZE: usize = 256;
const HALF_FFT: usize = FFT_SIZE / 2;
const PI: f32 = 3.14159265358979;

static mut NUM_CH: i32 = 0;

// FFT working buffers (one channel at a time)
static mut REAL: [f32; FFT_SIZE] = [0.0; FFT_SIZE];
static mut IMAG: [f32; FFT_SIZE] = [0.0; FFT_SIZE];

// Input accumulation buffer per channel
static mut ACCUM: [[f32; FFT_SIZE]; 8] = [[0.0; FFT_SIZE]; 8];
static mut ACCUM_POS: [usize; 8] = [0; 8];

// Output magnitudes (128 bins per channel)
static mut MAGNITUDES: [f32; HALF_FFT] = [0.0; HALF_FFT];

// Twiddle factors (precomputed in init)
static mut COS_TABLE: [f32; HALF_FFT] = [0.0; HALF_FFT];
static mut SIN_TABLE: [f32; HALF_FFT] = [0.0; HALF_FFT];

// Temporary input read buffer
static mut INPUT: [f32; FFT_SIZE] = [0.0; FFT_SIZE];

#[no_mangle]
pub extern "C" fn init() -> i32 {
    unsafe {
        NUM_CH = folk_daq_channel_count();

        // Precompute twiddle factors: e^(-j*2*pi*k/N) for k = 0..N/2-1
        for k in 0..HALF_FFT {
            let angle = -2.0 * PI * (k as f32) / (FFT_SIZE as f32);
            COS_TABLE[k] = cos_approx(angle);
            SIN_TABLE[k] = sin_approx(angle);
        }

        // Reset accumulation buffers
        for ch in 0..8 {
            ACCUM_POS[ch] = 0;
        }

        let msg = b"FFT: 256-point analyzer initialized";
        folk_log(msg.as_ptr() as i32, msg.len() as i32);
    }
    0
}

#[no_mangle]
pub extern "C" fn tick() -> i32 {
    unsafe {
        let avail = folk_daq_available();
        if avail <= 0 { return 0; }
        let n = if avail > FFT_SIZE as i32 { FFT_SIZE as i32 } else { avail };

        for ch in 0..NUM_CH {
            let read = folk_daq_read_samples(ch + 1, &raw const INPUT as i32, n);
            if read <= 0 { continue; }

            let ch_idx = ch as usize;

            // Accumulate samples until we have FFT_SIZE
            for i in 0..read as usize {
                if ACCUM_POS[ch_idx] < FFT_SIZE {
                    ACCUM[ch_idx][ACCUM_POS[ch_idx]] = INPUT[i];
                    ACCUM_POS[ch_idx] += 1;
                }
            }

            // When we have a full frame, compute FFT
            if ACCUM_POS[ch_idx] >= FFT_SIZE {
                // Copy to FFT working buffer
                for i in 0..FFT_SIZE {
                    REAL[i] = ACCUM[ch_idx][i];
                    IMAG[i] = 0.0;
                }

                // Apply Hanning window to reduce spectral leakage
                for i in 0..FFT_SIZE {
                    let w = 0.5 * (1.0 - cos_approx(2.0 * PI * (i as f32) / (FFT_SIZE as f32 - 1.0)));
                    REAL[i] *= w;
                }

                // In-place radix-2 FFT
                fft_radix2();

                // Compute magnitudes: |X[k]| = sqrt(re² + im²)
                for k in 0..HALF_FFT {
                    let re = REAL[k];
                    let im = IMAG[k];
                    MAGNITUDES[k] = sqrt_approx(re * re + im * im);
                }

                // Write spectrum to output
                folk_daq_write_output(ch + 1, &raw const MAGNITUDES as i32, HALF_FFT as i32);

                // Reset accumulator for next frame
                ACCUM_POS[ch_idx] = 0;
            }
        }
    }
    0
}

/// Radix-2 Decimation-in-Time FFT (in-place, iterative)
unsafe fn fft_radix2() {
    let n = FFT_SIZE;

    // Bit-reversal permutation
    let mut j: usize = 0;
    for i in 0..n {
        if i < j {
            // Swap real and imaginary parts
            let tmp_r = REAL[i]; REAL[i] = REAL[j]; REAL[j] = tmp_r;
            let tmp_i = IMAG[i]; IMAG[i] = IMAG[j]; IMAG[j] = tmp_i;
        }
        let mut m = n >> 1;
        while m >= 1 && j >= m {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    // Butterfly stages
    let mut stage_size: usize = 2;
    while stage_size <= n {
        let half = stage_size >> 1;
        let step = n / stage_size; // twiddle step

        let mut group = 0;
        while group < n {
            for k in 0..half {
                let tw_idx = k * step;
                let cos_tw = COS_TABLE[tw_idx];
                let sin_tw = SIN_TABLE[tw_idx];

                let even = group + k;
                let odd = even + half;

                // Butterfly: twiddle * odd
                let tr = cos_tw * REAL[odd] - sin_tw * IMAG[odd];
                let ti = sin_tw * REAL[odd] + cos_tw * IMAG[odd];

                REAL[odd] = REAL[even] - tr;
                IMAG[odd] = IMAG[even] - ti;
                REAL[even] += tr;
                IMAG[even] += ti;
            }
            group += stage_size;
        }
        stage_size <<= 1;
    }
}

// === Approximate math functions (no libm in no_std WASM) ===

/// Fast sine approximation using Bhaskara I formula
fn sin_approx(x: f32) -> f32 {
    // Normalize to [-π, π]
    let mut a = x;
    while a > PI { a -= 2.0 * PI; }
    while a < -PI { a += 2.0 * PI; }

    // Bhaskara approximation: sin(x) ≈ 16x(π-x) / (5π²-4x(π-x))
    let num = 16.0 * a * (PI - a);
    let den = 5.0 * PI * PI - 4.0 * a * (PI - a);
    if den == 0.0 { return 0.0; }
    num / den
}

fn cos_approx(x: f32) -> f32 {
    sin_approx(x + PI / 2.0)
}

/// Fast inverse square root (Quake-style) then invert
fn sqrt_approx(x: f32) -> f32 {
    if x <= 0.0 { return 0.0; }
    // Newton's method: start with rough estimate
    let mut guess = x * 0.5;
    // 3 iterations of Newton's: guess = (guess + x/guess) / 2
    guess = (guess + x / guess) * 0.5;
    guess = (guess + x / guess) * 0.5;
    guess = (guess + x / guess) * 0.5;
    guess
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
