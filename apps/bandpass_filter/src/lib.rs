//! Bandpass Filter — Cascaded IIR for Folkering DAQ
//!
//! Implements a bandpass filter by cascading a high-pass and low-pass
//! first-order IIR filter. Passes frequencies between f_low and f_high.
//!
//! High-pass: y[n] = α_hp * (y[n-1] + x[n] - x[n-1])
//! Low-pass:  y[n] = α_lp * x[n] + (1 - α_lp) * y[n-1]
//!
//! Default band: 200 Hz — 2000 Hz

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
const PI: f32 = 3.14159265;
const F_LOW: f32 = 200.0;
const F_HIGH: f32 = 2000.0;

// Filter state per channel
#[derive(Clone, Copy)]
struct ChannelState {
    // High-pass state
    hp_prev_x: f32,
    hp_prev_y: f32,
    // Low-pass state
    lp_prev_y: f32,
}

static mut ALPHA_HP: f32 = 0.0;
static mut ALPHA_LP: f32 = 0.0;
static mut CHANNELS: [ChannelState; 8] = [ChannelState {
    hp_prev_x: 0.0, hp_prev_y: 0.0, lp_prev_y: 0.0,
}; 8];
static mut NUM_CH: i32 = 0;
static mut INPUT: [f32; BATCH] = [0.0; BATCH];
static mut OUTPUT: [f32; BATCH] = [0.0; BATCH];

#[no_mangle]
pub extern "C" fn init() -> i32 {
    unsafe {
        let fs = folk_daq_sample_rate() as f32;
        NUM_CH = folk_daq_channel_count();

        // High-pass coefficient: α = RC / (RC + dt)
        let dt = 1.0 / fs;
        let rc_hp = 1.0 / (2.0 * PI * F_LOW);
        ALPHA_HP = rc_hp / (rc_hp + dt);

        // Low-pass coefficient: α = dt / (RC + dt)
        let rc_lp = 1.0 / (2.0 * PI * F_HIGH);
        ALPHA_LP = dt / (rc_lp + dt);

        let msg = b"BPF: 200-2000 Hz initialized";
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

        let a_hp = ALPHA_HP;
        let a_lp = ALPHA_LP;

        for ch in 0..NUM_CH {
            let read = folk_daq_read_samples(ch + 1, &raw const INPUT as i32, n);
            if read <= 0 { continue; }

            let st = &mut CHANNELS[ch as usize];

            for i in 0..read as usize {
                let x = INPUT[i];

                // Stage 1: High-pass
                let hp_y = a_hp * (st.hp_prev_y + x - st.hp_prev_x);
                st.hp_prev_x = x;
                st.hp_prev_y = hp_y;

                // Stage 2: Low-pass on the high-passed signal
                let lp_y = a_lp * hp_y + (1.0 - a_lp) * st.lp_prev_y;
                st.lp_prev_y = lp_y;

                OUTPUT[i] = lp_y;
            }

            folk_daq_write_output(ch + 1, &raw const OUTPUT as i32, read);
        }
    }
    0
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
