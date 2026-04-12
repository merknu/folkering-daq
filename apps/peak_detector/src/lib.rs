//! Peak Detector — Amplitude threshold monitor for Folkering DAQ
//!
//! Monitors all ADC channels for samples exceeding a configurable threshold.
//! When a peak is detected:
//!   1. The peak value and channel are logged to serial console
//!   2. The output ring buffer receives 1.0 for "peak" and 0.0 for "no peak"
//!      (binary event stream — useful for triggering downstream actions)
//!
//! The detector uses a simple envelope follower with configurable
//! attack/release time constants to avoid triggering on noise.
//!
//! Default threshold: 0.8 (80% of full-scale normalized ±1.0)
//! Default attack:    1 ms
//! Default release:  50 ms

#![no_std]

extern "C" {
    fn folk_daq_read_samples(channel: i32, buf_ptr: i32, max_samples: i32) -> i32;
    fn folk_daq_available() -> i32;
    fn folk_daq_sample_rate() -> i32;
    fn folk_daq_channel_count() -> i32;
    fn folk_daq_write_output(channel: i32, buf_ptr: i32, num_samples: i32) -> i32;
    fn folk_log(ptr: i32, len: i32) -> i32;
}

const BATCH: usize = 512;
const THRESHOLD: f32 = 0.8;
const ATTACK_MS: f32 = 1.0;
const RELEASE_MS: f32 = 50.0;

#[derive(Clone, Copy)]
struct ChannelState {
    envelope: f32,
    peak_active: bool,
    peak_count: u32,
}

static mut ATTACK_COEFF: f32 = 0.0;
static mut RELEASE_COEFF: f32 = 0.0;
static mut NUM_CH: i32 = 0;
static mut CHANNELS: [ChannelState; 8] = [ChannelState {
    envelope: 0.0, peak_active: false, peak_count: 0,
}; 8];
static mut INPUT: [f32; BATCH] = [0.0; BATCH];
static mut OUTPUT: [f32; BATCH] = [0.0; BATCH];
static mut TOTAL_PEAKS: u64 = 0;

#[no_mangle]
pub extern "C" fn init() -> i32 {
    unsafe {
        let fs = folk_daq_sample_rate() as f32;
        NUM_CH = folk_daq_channel_count();

        // Envelope follower coefficients
        // attack:  fast rise (small time constant → large coefficient)
        // release: slow fall (large time constant → small coefficient)
        let dt = 1.0 / fs;
        ATTACK_COEFF = dt / (ATTACK_MS / 1000.0 + dt);
        RELEASE_COEFF = dt / (RELEASE_MS / 1000.0 + dt);

        TOTAL_PEAKS = 0;

        let msg = b"PEAK: threshold=0.80 initialized";
        folk_log(msg.as_ptr() as i32, msg.len() as i32);
    }
    0
}

#[no_mangle]
pub extern "C" fn tick() -> i32 {
    unsafe {
        let avail = folk_daq_available();
        if avail <= 0 { return 0; }
        let n = if avail > BATCH as i32 { BATCH as i32 } else { avail };

        let att = ATTACK_COEFF;
        let rel = RELEASE_COEFF;

        for ch in 0..NUM_CH {
            let read = folk_daq_read_samples(ch + 1, &raw const INPUT as i32, n);
            if read <= 0 { continue; }

            let st = &mut CHANNELS[ch as usize];

            for i in 0..read as usize {
                let x = INPUT[i];

                // Absolute value
                let abs_x = if x < 0.0 { -x } else { x };

                // Envelope follower: fast attack, slow release
                if abs_x > st.envelope {
                    st.envelope += att * (abs_x - st.envelope);
                } else {
                    st.envelope += rel * (abs_x - st.envelope);
                }

                // Threshold crossing detection
                if st.envelope >= THRESHOLD {
                    if !st.peak_active {
                        st.peak_active = true;
                        st.peak_count += 1;
                        TOTAL_PEAKS += 1;
                    }
                    OUTPUT[i] = 1.0; // Peak event
                } else {
                    st.peak_active = false;
                    OUTPUT[i] = 0.0; // No peak
                }
            }

            folk_daq_write_output(ch + 1, &raw const OUTPUT as i32, read);
        }
    }
    0
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
