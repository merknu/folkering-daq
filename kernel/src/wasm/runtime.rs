//! DAQ WASM Runtime — wasmi-based interpreter for DSP apps
//!
//! Ported from Folkering OS's PersistentWasmApp pattern.
//! Uses wasmi (industry-standard WASM interpreter) instead of
//! the experimental Silverfir-nano JIT compiler.
//!
//! Lifecycle:
//!   1. DaqWasmApp::new(wasm_bytes) → parse + link + instantiate
//!   2. app.call_init() → call WASM "init" export (one-time setup)
//!   3. app.tick() → call WASM "tick" export (every main loop iteration)
//!   4. Hot-swap: replace with new DaqWasmApp, old is dropped
//!
//! Host functions (folk_daq_*) are registered via wasmi's Linker,
//! which handles all import resolution, type checking, and dispatch.

extern crate alloc;

use alloc::string::String;
use alloc::format;
use wasmi::*;

/// Fuel budget per tick (instructions). DSP apps are tight loops,
/// so 5M instructions should be more than enough for 512-sample batches.
const FUEL_PER_TICK: u64 = 5_000_000;

/// Shared state between host functions and the WASM module.
/// Host function closures access this via `caller.data()` / `caller.data_mut()`.
pub struct DaqHostState {
    /// Log messages from folk_log (collected per tick, flushed to serial)
    pub log_messages: alloc::vec::Vec<String>,
}

impl DaqHostState {
    fn new() -> Self {
        DaqHostState {
            log_messages: alloc::vec::Vec::new(),
        }
    }
}

/// Execution result
pub enum DaqWasmResult {
    Ok,
    OutOfFuel,
    Trap(String),
    LoadError(String),
}

/// A persistent WASM app for DAQ signal processing.
pub struct DaqWasmApp {
    store: Store<DaqHostState>,
    instance: Instance,
    tick_fn: Option<TypedFunc<(), i32>>,
    init_fn: Option<TypedFunc<(), i32>>,
    pub tick_count: u64,
    pub healthy: bool,
}

impl DaqWasmApp {
    /// Load, link, and instantiate a WASM module.
    pub fn new(wasm_bytes: &[u8]) -> Result<Self, String> {
        let engine = Engine::default();

        let module = Module::new(&engine, wasm_bytes)
            .map_err(|e| format!("WASM parse error: {:?}", e))?;

        let mut store = Store::new(&engine, DaqHostState::new());
        store.set_fuel(FUEL_PER_TICK).unwrap_or(());

        let mut linker = Linker::<DaqHostState>::new(&engine);
        super::host_functions::register_daq_functions(&mut linker);

        let pre_instance = linker.instantiate(&mut store, &module)
            .map_err(|e| format!("Instantiation error: {:?}", e))?;

        // Start the module (ensure no _start function, or run it)
        let instance = pre_instance.ensure_no_start(&mut store)
            .map_err(|e| format!("Start error: {:?}", e))?;

        // Look up exports by name
        let tick_fn = instance.get_export(&store, "tick")
            .and_then(|e| e.into_func())
            .and_then(|f| f.typed::<(), i32>(&store).ok());

        let init_fn = instance.get_export(&store, "init")
            .and_then(|e| e.into_func())
            .and_then(|f| f.typed::<(), i32>(&store).ok());

        if tick_fn.is_none() && init_fn.is_none() {
            return Err(String::from("WASM module has no 'init' or 'tick' export"));
        }

        Ok(DaqWasmApp {
            store,
            instance,
            tick_fn,
            init_fn,
            tick_count: 0,
            healthy: true,
        })
    }

    /// Call the "init" export (one-time setup).
    pub fn call_init(&mut self) -> DaqWasmResult {
        let init_fn = match &self.init_fn {
            Some(f) => f.clone(),
            None => return DaqWasmResult::Ok, // No init export, that's fine
        };

        self.store.set_fuel(FUEL_PER_TICK).unwrap_or(());

        match init_fn.call(&mut self.store, ()) {
            Ok(_result) => {
                self.flush_logs();
                DaqWasmResult::Ok
            }
            Err(e) => {
                self.healthy = false;
                let msg = format!("{:?}", e);
                self.flush_logs();
                if msg.contains("fuel") || msg.contains("Fuel") {
                    DaqWasmResult::OutOfFuel
                } else {
                    DaqWasmResult::Trap(msg)
                }
            }
        }
    }

    /// Call the "tick" export (every main loop iteration).
    pub fn tick(&mut self) -> DaqWasmResult {
        if !self.healthy {
            return DaqWasmResult::Trap(String::from("module in trapped state"));
        }

        let tick_fn = match &self.tick_fn {
            Some(f) => f.clone(),
            None => return DaqWasmResult::Ok,
        };

        self.store.set_fuel(FUEL_PER_TICK).unwrap_or(());

        match tick_fn.call(&mut self.store, ()) {
            Ok(_result) => {
                self.tick_count += 1;
                self.flush_logs();
                DaqWasmResult::Ok
            }
            Err(e) => {
                self.healthy = false;
                let msg = format!("{:?}", e);
                self.flush_logs();
                if msg.contains("fuel") || msg.contains("Fuel") {
                    DaqWasmResult::OutOfFuel
                } else {
                    DaqWasmResult::Trap(msg)
                }
            }
        }
    }

    /// Flush any log messages collected during execution
    fn flush_logs(&mut self) {
        let state = self.store.data_mut();
        for msg in state.log_messages.drain(..) {
            crate::kprintln!("[WASM] {}", msg);
        }
    }

    /// Check if module is healthy
    pub fn is_healthy(&self) -> bool {
        self.healthy
    }

    /// Get tick count
    pub fn tick_count(&self) -> u64 {
        self.tick_count
    }
}
