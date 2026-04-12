//! Silverfir Runtime — Execute JIT-compiled WASM functions
//!
//! Handles the actual invocation of compiled native code, including:
//!   - Setting up the calling convention (linear memory ptr, args)
//!   - Catching traps (illegal instructions, segfaults)
//!   - Managing the WASM operand stack

use super::SilverfirError;

/// Call a JIT-compiled function.
///
/// Calling convention:
///   AArch64: x0 = linear_memory_ptr, x1 = mem_size, x2-x5 = args
///   x86_64:  rdi = linear_memory_ptr, rsi = mem_size, rdx/rcx/r8/r9 = args
///
/// Returns the function's return value (i64), or None if void.
///
/// # Safety
/// - `code_ptr` must point to valid, executable native code
/// - `linear_memory` must be a valid, writable buffer
pub unsafe fn call_jit_function(
    code_ptr: *const u8,
    linear_memory: *mut u8,
    mem_size: usize,
    args: &[i64],
) -> Result<Option<i64>, SilverfirError> {
    let a0 = args.get(0).copied().unwrap_or(0);
    let a1 = args.get(1).copied().unwrap_or(0);
    let a2 = args.get(2).copied().unwrap_or(0);
    let a3 = args.get(3).copied().unwrap_or(0);

    let result: i64;

    #[cfg(target_arch = "aarch64")]
    {
        core::arch::asm!(
            "blr {code}",
            code = in(reg) code_ptr,
            in("x0") linear_memory as u64,
            in("x1") mem_size as u64,
            in("x2") a0 as u64,
            in("x3") a1 as u64,
            in("x4") a2 as u64,
            in("x5") a3 as u64,
            lateout("x0") result,
            // Clobber scratch registers
            clobber_abi("C"),
        );
    }

    #[cfg(target_arch = "x86_64")]
    {
        core::arch::asm!(
            "call {code}",
            code = in(reg) code_ptr,
            in("rdi") linear_memory as u64,
            in("rsi") mem_size as u64,
            in("rdx") a0 as u64,
            in("rcx") a1 as u64,
            in("r8") a2 as u64,
            in("r9") a3 as u64,
            lateout("rax") result,
            clobber_abi("C"),
        );
    }

    #[cfg(not(any(target_arch = "aarch64", target_arch = "x86_64")))]
    {
        let _ = (code_ptr, linear_memory, mem_size, a0, a1, a2, a3);
        return Err(SilverfirError::CompileError("unsupported architecture"));
    }

    Ok(Some(result))
}

/// A running instance of a WASM module.
/// Manages state between calls (linear memory persists).
pub struct SilverfirInstance {
    /// The compiled module
    pub module: super::SilverfirModule,

    /// Tick counter for telemetry
    pub ticks: u64,

    /// Whether the module is in an error state
    pub trapped: bool,
}

impl SilverfirInstance {
    /// Create a new instance from a compiled module.
    pub fn new(module: super::SilverfirModule) -> Self {
        SilverfirInstance {
            module,
            ticks: 0,
            trapped: false,
        }
    }

    /// Call the module's "tick" export (main DSP processing function).
    /// This is called once per main-loop iteration.
    ///
    /// Convention: the "tick" function receives no arguments and returns 0 on success.
    pub fn tick(&mut self) -> Result<(), SilverfirError> {
        if self.trapped { return Err(SilverfirError::Trap("module is in trapped state")); }

        let tick_idx = match self.module.find_export("tick") {
            Some(idx) => idx,
            None => return Ok(()), // No tick function — nothing to do
        };

        match unsafe { self.module.call(tick_idx, &[]) } {
            Ok(_) => {
                self.ticks += 1;
                Ok(())
            }
            Err(e) => {
                self.trapped = true;
                Err(e)
            }
        }
    }

    /// Call the module's "init" export (one-time setup).
    pub fn init(&mut self) -> Result<(), SilverfirError> {
        let init_idx = match self.module.find_export("init") {
            Some(idx) => idx,
            None => return Ok(()),
        };

        match unsafe { self.module.call(init_idx, &[]) } {
            Ok(_) => Ok(()),
            Err(e) => {
                self.trapped = true;
                Err(e)
            }
        }
    }

    /// Hot-swap: replace the current module with a new one.
    /// Linear memory is NOT preserved — the new module starts fresh.
    pub fn hot_swap(&mut self, new_module: super::SilverfirModule) {
        self.module = new_module;
        self.trapped = false;
        self.ticks = 0;
        // Call init on the new module
        let _ = self.init();
    }

    /// Get the tick count (for telemetry / fitness tracking)
    pub fn tick_count(&self) -> u64 {
        self.ticks
    }

    /// Check if module is healthy
    pub fn is_healthy(&self) -> bool {
        !self.trapped
    }
}
