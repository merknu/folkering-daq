//! AArch64 Instruction Emitter for Silverfir-nano JIT
//!
//! Emits correctly encoded ARMv8-A (AArch64) machine code.
//! Each function takes operands and returns a 4-byte little-endian instruction.
//!
//! Encoding reference: ARM Architecture Reference Manual (ARMv8-A)
//! All instructions are 32 bits, little-endian.
//!
//! Register conventions for Silverfir JIT:
//!   X0  = linear memory base pointer (preserved across function)
//!   X1  = linear memory size
//!   X29 = frame pointer (FP)
//!   X30 = link register (LR)
//!   SP  = stack pointer (operand stack for WASM)
//!   W9-W15  = scratch registers (i32 operands)
//!   S0-S7   = scratch FP registers (f32 operands)
//!   X19-X28 = callee-saved (used for locals if needed)

use alloc::vec::Vec;

/// Rd, Rn, Rm are 5-bit register indices (0-30, 31=SP/ZR)
type Reg = u8;

// Named registers
pub const X0: Reg = 0;
pub const X1: Reg = 1;
pub const X29: Reg = 29; // FP
pub const X30: Reg = 30; // LR
pub const XZR: Reg = 31; // Zero register
pub const SP: Reg = 31;  // Stack pointer (same encoding, context-dependent)

pub const W9: Reg = 9;
pub const W10: Reg = 10;
pub const W11: Reg = 11;

pub const S0: Reg = 0;
pub const S1: Reg = 1;

// ── Prologue / Epilogue ─────────────────────────────────────────

/// STP X29, X30, [SP, #-16]! — save frame pointer and link register
pub fn stp_pre(rt1: Reg, rt2: Reg, rn: Reg, imm7: i16) -> u32 {
    let opc = 0b10; // 64-bit
    let imm = ((imm7 / 8) as u32) & 0x7F;
    (opc << 30) | (0b101 << 27) | (0b0 << 26) | (0b011 << 23) // pre-index
        | (imm << 15) | ((rt2 as u32) << 10) | ((rn as u32) << 5) | (rt1 as u32)
}

/// LDP X29, X30, [SP], #16 — restore frame pointer and link register
pub fn ldp_post(rt1: Reg, rt2: Reg, rn: Reg, imm7: i16) -> u32 {
    let opc = 0b10; // 64-bit
    let imm = ((imm7 / 8) as u32) & 0x7F;
    (opc << 30) | (0b101 << 27) | (0b0 << 26) | (0b001 << 23) // post-index
        | (imm << 15) | ((rt2 as u32) << 10) | ((rn as u32) << 5) | (rt1 as u32)
}

/// Standard function prologue
pub fn prologue() -> Vec<u32> {
    alloc::vec![
        stp_pre(X29, X30, SP, -16),   // STP X29, X30, [SP, #-16]!
        add_imm64(X29, SP, 0),         // MOV X29, SP
    ]
}

/// Standard function epilogue
pub fn epilogue() -> Vec<u32> {
    alloc::vec![
        ldp_post(X29, X30, SP, 16),   // LDP X29, X30, [SP], #16
        ret(),                          // RET
    ]
}

// ── Integer Arithmetic (32-bit W registers) ─────────────────────

/// ADD Wd, Wn, Wm
pub fn add_reg(rd: Reg, rn: Reg, rm: Reg) -> u32 {
    0x0B000000 | ((rm as u32) << 16) | ((rn as u32) << 5) | (rd as u32)
}

/// SUB Wd, Wn, Wm
pub fn sub_reg(rd: Reg, rn: Reg, rm: Reg) -> u32 {
    0x4B000000 | ((rm as u32) << 16) | ((rn as u32) << 5) | (rd as u32)
}

/// MUL Wd, Wn, Wm  (alias for MADD Wd, Wn, Wm, WZR)
pub fn mul_reg(rd: Reg, rn: Reg, rm: Reg) -> u32 {
    0x1B007C00 | ((rm as u32) << 16) | ((rn as u32) << 5) | (rd as u32)
}

/// SDIV Wd, Wn, Wm
pub fn sdiv_reg(rd: Reg, rn: Reg, rm: Reg) -> u32 {
    0x1AC00C00 | ((rm as u32) << 16) | ((rn as u32) << 5) | (rd as u32)
}

/// AND Wd, Wn, Wm
pub fn and_reg(rd: Reg, rn: Reg, rm: Reg) -> u32 {
    0x0A000000 | ((rm as u32) << 16) | ((rn as u32) << 5) | (rd as u32)
}

/// ORR Wd, Wn, Wm
pub fn orr_reg(rd: Reg, rn: Reg, rm: Reg) -> u32 {
    0x2A000000 | ((rm as u32) << 16) | ((rn as u32) << 5) | (rd as u32)
}

/// EOR Wd, Wn, Wm (XOR)
pub fn eor_reg(rd: Reg, rn: Reg, rm: Reg) -> u32 {
    0x4A000000 | ((rm as u32) << 16) | ((rn as u32) << 5) | (rd as u32)
}

/// LSL Wd, Wn, Wm (alias for LSLV)
pub fn lsl_reg(rd: Reg, rn: Reg, rm: Reg) -> u32 {
    0x1AC02000 | ((rm as u32) << 16) | ((rn as u32) << 5) | (rd as u32)
}

/// ASR Wd, Wn, Wm (arithmetic shift right, alias for ASRV)
pub fn asr_reg(rd: Reg, rn: Reg, rm: Reg) -> u32 {
    0x1AC02800 | ((rm as u32) << 16) | ((rn as u32) << 5) | (rd as u32)
}

/// CMP Wn, Wm (alias for SUBS WZR, Wn, Wm)
pub fn cmp_reg(rn: Reg, rm: Reg) -> u32 {
    0x6B000000 | ((rm as u32) << 16) | ((rn as u32) << 5) | (XZR as u32)
}

/// CSET Wd, <cond> (set register to 1 if condition true, else 0)
/// cond: 0=EQ, 1=NE, 10=GE, 11=LT, 12=GT, 13=LE
pub fn cset(rd: Reg, cond: u8) -> u32 {
    // CSET is alias for CSINC Wd, WZR, WZR, invert(cond)
    let inv_cond = (cond ^ 1) as u32; // Invert lowest bit
    0x1A9F0400 | (inv_cond << 12) | ((XZR as u32) << 16) | ((XZR as u32) << 5) | (rd as u32)
}

// Condition codes
pub const COND_EQ: u8 = 0;
pub const COND_NE: u8 = 1;
pub const COND_LT: u8 = 11;
pub const COND_GE: u8 = 10;
pub const COND_GT: u8 = 12;
pub const COND_LE: u8 = 13;

// ── Immediate Operations ────────────────────────────────────────

/// ADD Xd, Xn, #imm12 (64-bit, for SP manipulation)
pub fn add_imm64(rd: Reg, rn: Reg, imm12: u16) -> u32 {
    0x91000000 | (((imm12 as u32) & 0xFFF) << 10) | ((rn as u32) << 5) | (rd as u32)
}

/// SUB Xd, Xn, #imm12 (64-bit)
pub fn sub_imm64(rd: Reg, rn: Reg, imm12: u16) -> u32 {
    0xD1000000 | (((imm12 as u32) & 0xFFF) << 10) | ((rn as u32) << 5) | (rd as u32)
}

/// ADD Wd, Wn, #imm12 (32-bit)
pub fn add_imm32(rd: Reg, rn: Reg, imm12: u16) -> u32 {
    0x11000000 | (((imm12 as u32) & 0xFFF) << 10) | ((rn as u32) << 5) | (rd as u32)
}

/// MOVZ Wd, #imm16 (move wide immediate, zero others)
pub fn movz_w(rd: Reg, imm16: u16) -> u32 {
    0x52800000 | ((imm16 as u32) << 5) | (rd as u32)
}

/// MOVK Wd, #imm16, LSL #16 (keep lower 16 bits, set upper 16)
pub fn movk_w_16(rd: Reg, imm16: u16) -> u32 {
    0x72A00000 | ((imm16 as u32) << 5) | (rd as u32)
}

/// Load a full 32-bit immediate into Wd (1 or 2 instructions)
pub fn mov_imm32(rd: Reg, val: u32) -> Vec<u32> {
    let lo = (val & 0xFFFF) as u16;
    let hi = ((val >> 16) & 0xFFFF) as u16;

    if hi == 0 {
        alloc::vec![movz_w(rd, lo)]
    } else if lo == 0 {
        alloc::vec![
            0x52800000 | (rd as u32), // MOVZ Wd, #0
            movk_w_16(rd, hi),
        ]
    } else {
        alloc::vec![movz_w(rd, lo), movk_w_16(rd, hi)]
    }
}

// ── Load / Store ────────────────────────────────────────────────

/// LDR Wt, [Xn, Xm] — load 32-bit from base + offset register
pub fn ldr_reg_w(rt: Reg, rn: Reg, rm: Reg) -> u32 {
    0xB8606800 | ((rm as u32) << 16) | ((rn as u32) << 5) | (rt as u32)
}

/// STR Wt, [Xn, Xm] — store 32-bit to base + offset register
pub fn str_reg_w(rt: Reg, rn: Reg, rm: Reg) -> u32 {
    0xB8206800 | ((rm as u32) << 16) | ((rn as u32) << 5) | (rt as u32)
}

/// LDR Wt, [Xn, #imm12] — load 32-bit unsigned offset (scaled by 4)
pub fn ldr_imm_w(rt: Reg, rn: Reg, offset: u16) -> u32 {
    let scaled = (offset / 4) as u32;
    0xB9400000 | ((scaled & 0xFFF) << 10) | ((rn as u32) << 5) | (rt as u32)
}

/// STR Wt, [Xn, #imm12] — store 32-bit unsigned offset (scaled by 4)
pub fn str_imm_w(rt: Reg, rn: Reg, offset: u16) -> u32 {
    let scaled = (offset / 4) as u32;
    0xB9000000 | ((scaled & 0xFFF) << 10) | ((rn as u32) << 5) | (rt as u32)
}

/// LDR Wt, [SP, #imm12] — load from stack frame
pub fn ldr_sp_w(rt: Reg, offset: u16) -> u32 {
    ldr_imm_w(rt, SP, offset)
}

/// STR Wt, [SP, #imm12] — store to stack frame
pub fn str_sp_w(rt: Reg, offset: u16) -> u32 {
    str_imm_w(rt, SP, offset)
}

/// STR Wt, [SP, #-4]! — push 32-bit to stack (pre-decrement)
pub fn push_w(rt: Reg) -> u32 {
    // STR Wt, [SP, #-4]! = STUR with pre-index
    0xB81FC000 | ((SP as u32) << 5) | (rt as u32)
}

/// LDR Wt, [SP], #4 — pop 32-bit from stack (post-increment)
pub fn pop_w(rt: Reg) -> u32 {
    // LDR Wt, [SP], #4 = LDUR with post-index
    0xB8404000 | ((SP as u32) << 5) | (rt as u32)
}

// ── Floating-Point (32-bit S registers) ─────────────────────────

/// FADD Sd, Sn, Sm
pub fn fadd_s(rd: Reg, rn: Reg, rm: Reg) -> u32 {
    0x1E202800 | ((rm as u32) << 16) | ((rn as u32) << 5) | (rd as u32)
}

/// FSUB Sd, Sn, Sm
pub fn fsub_s(rd: Reg, rn: Reg, rm: Reg) -> u32 {
    0x1E203800 | ((rm as u32) << 16) | ((rn as u32) << 5) | (rd as u32)
}

/// FMUL Sd, Sn, Sm
pub fn fmul_s(rd: Reg, rn: Reg, rm: Reg) -> u32 {
    0x1E200800 | ((rm as u32) << 16) | ((rn as u32) << 5) | (rd as u32)
}

/// FDIV Sd, Sn, Sm
pub fn fdiv_s(rd: Reg, rn: Reg, rm: Reg) -> u32 {
    0x1E201800 | ((rm as u32) << 16) | ((rn as u32) << 5) | (rd as u32)
}

/// FABS Sd, Sn
pub fn fabs_s(rd: Reg, rn: Reg) -> u32 {
    0x1E20C000 | ((rn as u32) << 5) | (rd as u32)
}

/// FNEG Sd, Sn
pub fn fneg_s(rd: Reg, rn: Reg) -> u32 {
    0x1E214000 | ((rn as u32) << 5) | (rd as u32)
}

/// FSQRT Sd, Sn
pub fn fsqrt_s(rd: Reg, rn: Reg) -> u32 {
    0x1E21C000 | ((rn as u32) << 5) | (rd as u32)
}

/// FCVTZS Wd, Sn (float → int, truncate toward zero)
pub fn fcvtzs_ws(rd: Reg, rn: Reg) -> u32 {
    0x1E380000 | ((rn as u32) << 5) | (rd as u32)
}

/// SCVTF Sd, Wn (int → float)
pub fn scvtf_sw(rd: Reg, rn: Reg) -> u32 {
    0x1E220000 | ((rn as u32) << 5) | (rd as u32)
}

/// LDR St, [Xn, #imm12] — load f32 from memory (scaled by 4)
pub fn ldr_imm_s(rt: Reg, rn: Reg, offset: u16) -> u32 {
    let scaled = (offset / 4) as u32;
    0xBD400000 | ((scaled & 0xFFF) << 10) | ((rn as u32) << 5) | (rt as u32)
}

/// STR St, [Xn, #imm12] — store f32 to memory (scaled by 4)
pub fn str_imm_s(rt: Reg, rn: Reg, offset: u16) -> u32 {
    let scaled = (offset / 4) as u32;
    0xBD000000 | ((scaled & 0xFFF) << 10) | ((rn as u32) << 5) | (rt as u32)
}

/// FMOV Wd, Sn — move FP to integer register (bit copy)
pub fn fmov_ws(rd: Reg, rn: Reg) -> u32 {
    0x1E260000 | ((rn as u32) << 5) | (rd as u32)
}

/// FMOV Sd, Wn — move integer to FP register (bit copy)
pub fn fmov_sw(rd: Reg, rn: Reg) -> u32 {
    0x1E270000 | ((rn as u32) << 5) | (rd as u32)
}

// ── Branching ───────────────────────────────────────────────────

/// B #offset — unconditional branch (PC-relative, offset in bytes, div by 4)
pub fn b(byte_offset: i32) -> u32 {
    let imm26 = ((byte_offset >> 2) as u32) & 0x03FF_FFFF;
    0x14000000 | imm26
}

/// BL #offset — branch with link (call)
pub fn bl(byte_offset: i32) -> u32 {
    let imm26 = ((byte_offset >> 2) as u32) & 0x03FF_FFFF;
    0x94000000 | imm26
}

/// BLR Xn — branch with link to register (indirect call)
pub fn blr(rn: Reg) -> u32 {
    0xD63F0000 | ((rn as u32) << 5)
}

/// BR Xn — branch to register (indirect jump)
pub fn br(rn: Reg) -> u32 {
    0xD61F0000 | ((rn as u32) << 5)
}

/// RET — return (alias for BR X30)
pub fn ret() -> u32 {
    0xD65F03C0
}

/// CBZ Wt, #offset — compare and branch if zero
pub fn cbz_w(rt: Reg, byte_offset: i32) -> u32 {
    let imm19 = ((byte_offset >> 2) as u32) & 0x7FFFF;
    0x34000000 | (imm19 << 5) | (rt as u32)
}

/// CBNZ Wt, #offset — compare and branch if non-zero
pub fn cbnz_w(rt: Reg, byte_offset: i32) -> u32 {
    let imm19 = ((byte_offset >> 2) as u32) & 0x7FFFF;
    0x35000000 | (imm19 << 5) | (rt as u32)
}

/// B.cond #offset — conditional branch
pub fn b_cond(cond: u8, byte_offset: i32) -> u32 {
    let imm19 = ((byte_offset >> 2) as u32) & 0x7FFFF;
    0x54000000 | (imm19 << 5) | (cond as u32)
}

// ── Miscellaneous ───────────────────────────────────────────────

/// NOP
pub fn nop() -> u32 {
    0xD503201F
}

/// BRK #imm16 — breakpoint (trap)
pub fn brk(imm16: u16) -> u32 {
    0xD4200000 | ((imm16 as u32) << 5)
}

/// MOV Xd, Xn (alias for ORR Xd, XZR, Xn)
pub fn mov_x(rd: Reg, rn: Reg) -> u32 {
    0xAA0003E0 | ((rn as u32) << 16) | (rd as u32)
}

/// MOV Wd, Wn (alias for ORR Wd, WZR, Wn)
pub fn mov_w(rd: Reg, rn: Reg) -> u32 {
    0x2A0003E0 | ((rn as u32) << 16) | (rd as u32)
}

// ── Helper: Encode instruction to bytes ─────────────────────────

/// Encode a single instruction to 4 little-endian bytes
pub fn encode(inst: u32) -> [u8; 4] {
    inst.to_le_bytes()
}

/// Encode a sequence of instructions to bytes
pub fn encode_all(instructions: &[u32]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(instructions.len() * 4);
    for &inst in instructions {
        bytes.extend_from_slice(&inst.to_le_bytes());
    }
    bytes
}
