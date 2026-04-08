//! AArch64 Exception Vector Table
//!
//! ARM64 has 4 exception types × 4 source contexts = 16 vectors.
//! Each vector entry is 128 bytes (0x80, 32 instructions max).
//!
//! Exception types: Synchronous, IRQ, FIQ, SError
//! Contexts: Current EL SP0, Current EL SPx, Lower EL AArch64, Lower EL AArch32
//!
//! The vector table must be 2048-byte (0x800) aligned.
//! VBAR_EL1 points to the table base.

use core::arch::global_asm;

/// Saved register context for exception handlers
#[repr(C)]
pub struct ExceptionContext {
    pub gpr: [u64; 31],   // x0-x30
    pub elr: u64,          // Exception Link Register
    pub spsr: u64,         // Saved Program Status Register
}

/// Install the exception vector table into VBAR_EL1
pub fn init() {
    unsafe {
        core::arch::asm!(
            "adr {tmp}, exception_vector_table",
            "msr VBAR_EL1, {tmp}",
            "isb",
            tmp = out(reg) _,
            options(nomem, nostack)
        );
    }
}

// ============================================================
// Rust exception handlers (called from asm stubs)
// ============================================================

/// Read Exception Syndrome Register
#[inline(always)]
fn read_esr() -> u64 {
    let v: u64;
    unsafe { core::arch::asm!("mrs {0}, ESR_EL1", out(reg) v, options(nomem, nostack)); }
    v
}

/// Read Fault Address Register
#[inline(always)]
fn read_far() -> u64 {
    let v: u64;
    unsafe { core::arch::asm!("mrs {0}, FAR_EL1", out(reg) v, options(nomem, nostack)); }
    v
}

#[no_mangle]
extern "C" fn handle_sync_exception(ctx: &ExceptionContext) {
    let esr = read_esr();
    let ec = (esr >> 26) & 0x3F;
    let far = read_far();

    match ec {
        0x15 => {
            // SVC instruction — syscall
            crate::kprintln!("SYSCALL #{}", ctx.gpr[8]);
        }
        0x20 | 0x21 => {
            crate::kprintln!("INSTRUCTION ABORT at ELR={:#x}, FAR={:#x}", ctx.elr, far);
            loop { super::wfe(); }
        }
        0x24 | 0x25 => {
            crate::kprintln!("DATA ABORT at ELR={:#x}, FAR={:#x}, ESR={:#x}", ctx.elr, far, esr);
            loop { super::wfe(); }
        }
        _ => {
            crate::kprintln!("SYNC EXCEPTION: EC={:#x} ESR={:#x} ELR={:#x}", ec, esr, ctx.elr);
            loop { super::wfe(); }
        }
    }
}

#[no_mangle]
extern "C" fn handle_irq(_ctx: &ExceptionContext) {
    super::gic::handle_irq();
}

#[no_mangle]
extern "C" fn handle_serror(ctx: &ExceptionContext) {
    crate::kprintln!("SERROR at ELR={:#x}", ctx.elr);
    loop { super::wfe(); }
}

// ============================================================
// Exception Vector Table (assembly)
// ============================================================
global_asm!(
r"
.section .text

// Save 31 GPRs + ELR + SPSR = 33 × 8 = 264 bytes
.macro SAVE_REGS
    sub sp, sp, #(33 * 8)
    stp x0,  x1,  [sp, #(16 * 0)]
    stp x2,  x3,  [sp, #(16 * 1)]
    stp x4,  x5,  [sp, #(16 * 2)]
    stp x6,  x7,  [sp, #(16 * 3)]
    stp x8,  x9,  [sp, #(16 * 4)]
    stp x10, x11, [sp, #(16 * 5)]
    stp x12, x13, [sp, #(16 * 6)]
    stp x14, x15, [sp, #(16 * 7)]
    stp x16, x17, [sp, #(16 * 8)]
    stp x18, x19, [sp, #(16 * 9)]
    stp x20, x21, [sp, #(16 * 10)]
    stp x22, x23, [sp, #(16 * 11)]
    stp x24, x25, [sp, #(16 * 12)]
    stp x26, x27, [sp, #(16 * 13)]
    stp x28, x29, [sp, #(16 * 14)]
    str x30,      [sp, #(16 * 15)]
    mrs x21, ELR_EL1
    mrs x22, SPSR_EL1
    stp x21, x22, [sp, #(16 * 15 + 8)]
.endm

.macro RESTORE_REGS
    ldp x21, x22, [sp, #(16 * 15 + 8)]
    msr ELR_EL1, x21
    msr SPSR_EL1, x22
    ldp x0,  x1,  [sp, #(16 * 0)]
    ldp x2,  x3,  [sp, #(16 * 1)]
    ldp x4,  x5,  [sp, #(16 * 2)]
    ldp x6,  x7,  [sp, #(16 * 3)]
    ldp x8,  x9,  [sp, #(16 * 4)]
    ldp x10, x11, [sp, #(16 * 5)]
    ldp x12, x13, [sp, #(16 * 6)]
    ldp x14, x15, [sp, #(16 * 7)]
    ldp x16, x17, [sp, #(16 * 8)]
    ldp x18, x19, [sp, #(16 * 9)]
    ldp x20, x21, [sp, #(16 * 10)]
    ldp x22, x23, [sp, #(16 * 11)]
    ldp x24, x25, [sp, #(16 * 12)]
    ldp x26, x27, [sp, #(16 * 13)]
    ldp x28, x29, [sp, #(16 * 14)]
    ldr x30,      [sp, #(16 * 15)]
    add sp, sp, #(33 * 8)
    eret
.endm

.macro HANDLER_ENTRY handler
    .balign 0x80
    SAVE_REGS
    mov x0, sp
    bl \handler
    RESTORE_REGS
.endm

.macro HANDLER_UNUSED
    .balign 0x80
    wfe
    b -4
.endm

// 2048-byte aligned exception vector table
.balign 0x800
.global exception_vector_table
exception_vector_table:
    // Current EL with SP0 (not used)
    HANDLER_UNUSED
    HANDLER_UNUSED
    HANDLER_UNUSED
    HANDLER_UNUSED

    // Current EL with SPx (kernel)
    HANDLER_ENTRY handle_sync_exception
    HANDLER_ENTRY handle_irq
    HANDLER_UNUSED
    HANDLER_ENTRY handle_serror

    // Lower EL, AArch64 (userspace)
    HANDLER_ENTRY handle_sync_exception
    HANDLER_ENTRY handle_irq
    HANDLER_UNUSED
    HANDLER_ENTRY handle_serror

    // Lower EL, AArch32 (not supported)
    HANDLER_UNUSED
    HANDLER_UNUSED
    HANDLER_UNUSED
    HANDLER_UNUSED
"
);
