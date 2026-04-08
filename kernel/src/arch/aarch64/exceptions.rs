//! AArch64 Exception Vector Table
//!
//! ARM64 has 4 exception types × 4 source contexts = 16 vectors.
//! Each vector entry is 128 bytes (32 instructions).
//!
//! Exception types: Synchronous, IRQ, FIQ, SError
//! Source contexts: Current EL SP0, Current EL SPx, Lower EL AArch64, Lower EL AArch32

use core::arch::global_asm;

/// Install the exception vector table
pub fn init() {
    // Exception vector table will be set up when we have the proper .S file
    // For now, just log that we'd install VBAR_EL1
    // TODO: msr VBAR_EL1, <vector_table_addr>; isb
}

// Saved context for exception handlers
#[repr(C)]
pub struct ExceptionContext {
    pub gpr: [u64; 31],   // x0-x30
    pub elr: u64,          // Exception Link Register (return address)
    pub spsr: u64,         // Saved Program Status Register
    pub esr: u64,          // Exception Syndrome Register
    pub far: u64,          // Fault Address Register
}

/// Read Exception Syndrome Register
#[inline(always)]
fn read_esr_el1() -> u64 {
    let esr: u64;
    unsafe { core::arch::asm!("mrs {}, ESR_EL1", out(reg) esr, options(nomem, nostack)); }
    esr
}

/// Read Fault Address Register
#[inline(always)]
fn read_far_el1() -> u64 {
    let far: u64;
    unsafe { core::arch::asm!("mrs {}, FAR_EL1", out(reg) far, options(nomem, nostack)); }
    far
}

// Exception handlers (called from assembly stubs)

#[no_mangle]
extern "C" fn current_el_spx_sync(ctx: &ExceptionContext) {
    let esr = read_esr_el1();
    let ec = (esr >> 26) & 0x3F;
    let far = read_far_el1();

    match ec {
        0x15 => {
            // SVC from AArch64 — syscall
            // TODO: dispatch syscall
            crate::kprintln!("SYSCALL #{} from kernel", ctx.gpr[8]);
        }
        0x20 | 0x21 => {
            // Instruction abort
            crate::kprintln!("INSTRUCTION ABORT at {:#x}, FAR={:#x}", ctx.elr, far);
            loop { super::wfe(); }
        }
        0x24 | 0x25 => {
            // Data abort
            crate::kprintln!("DATA ABORT at {:#x}, FAR={:#x}, ESR={:#x}", ctx.elr, far, esr);
            loop { super::wfe(); }
        }
        _ => {
            crate::kprintln!("UNHANDLED SYNC EXCEPTION: EC={:#x}, ESR={:#x}, ELR={:#x}", ec, esr, ctx.elr);
            loop { super::wfe(); }
        }
    }
}

#[no_mangle]
extern "C" fn current_el_spx_irq(_ctx: &ExceptionContext) {
    // Dispatch IRQ via GIC
    super::gic::handle_irq();
}

#[no_mangle]
extern "C" fn current_el_spx_fiq(_ctx: &ExceptionContext) {
    crate::kprintln!("FIQ — unexpected");
}

#[no_mangle]
extern "C" fn current_el_spx_serror(ctx: &ExceptionContext) {
    crate::kprintln!("SERROR at {:#x}", ctx.elr);
    loop { super::wfe(); }
}

#[no_mangle]
extern "C" fn lower_el_aarch64_sync(ctx: &ExceptionContext) {
    let esr = read_esr_el1();
    let ec = (esr >> 26) & 0x3F;

    match ec {
        0x15 => {
            // SVC from userspace — syscall dispatch
            // syscall number in x8, args in x0-x5, return in x0
            // TODO: userspace syscall handler
            crate::kprintln!("USER SYSCALL #{}", ctx.gpr[8]);
        }
        _ => {
            crate::kprintln!("LOWER EL SYNC: EC={:#x}, ELR={:#x}", ec, ctx.elr);
            loop { super::wfe(); }
        }
    }
}

#[no_mangle]
extern "C" fn lower_el_aarch64_irq(_ctx: &ExceptionContext) {
    super::gic::handle_irq();
}

// Exception vector table will be in a separate .S file compiled with proper target
// For now, use a minimal Rust-only placeholder
static VECTOR_TABLE_PLACEHOLDER: u64 = 0;

extern "C" {
    fn exception_vector_table();
}

/// Placeholder — will be replaced with proper assembly file
#[no_mangle]
pub extern "C" fn exception_vector_table_stub() {
    // Actual vector table needs to be in a .S file
    // assembled with aarch64 assembler
}
