//! WASM → Native Code Compiler (Single-Pass)
//!
//! Translates WASM bytecode to native machine code in a single forward pass.
//! Uses aarch64_emit.rs for correct ARM64 instruction encoding.
//!
//! Strategy: Stack-machine emulation via native stack (SP).
//!   - WASM i32/f32 values live on the native stack as 4-byte slots
//!   - push/pop via STR/LDR with SP pre-decrement/post-increment
//!   - Binary ops: pop two, compute, push result
//!   - Locals stored in stack frame below FP
//!   - Linear memory accessed via X0 (base pointer, set by caller)
//!
//! Register allocation:
//!   X0  = linear memory base (preserved)
//!   X1  = linear memory size (preserved)
//!   W9  = scratch 0 (primary operand)
//!   W10 = scratch 1 (secondary operand)
//!   W11 = scratch 2 (address computation)
//!   S0  = FP scratch 0
//!   S1  = FP scratch 1
//!   X29 = frame pointer
//!   X30 = link register
//!   SP  = operand stack

use alloc::vec::Vec;
use super::SilverfirError;
use super::parser::FuncType;

#[cfg(target_arch = "aarch64")]
use super::aarch64_emit as arm;

/// Compile a single WASM function body to native code.
pub fn compile_function(
    body: &[u8],
    func_type: Option<&FuncType>,
) -> Result<Vec<u8>, SilverfirError> {
    let mut emit = Emitter::new();
    let mut reader = BodyReader::new(body);

    // Parse local declarations
    let local_decl_count = reader.read_u32_leb128()?;
    let mut total_locals: u32 = 0;
    for _ in 0..local_decl_count {
        let count = reader.read_u32_leb128()?;
        let _valtype = reader.read_byte()?;
        total_locals += count;
    }

    emit.emit_prologue(total_locals);

    // Translate opcodes
    while reader.remaining() > 0 {
        let opcode = reader.read_byte()?;
        match opcode {
            // === Control Flow ===
            0x00 => emit.emit_trap(),
            0x01 => {} // nop
            0x02 => { let _bt = reader.read_byte()?; emit.emit_block_start(false); }
            0x03 => { let _bt = reader.read_byte()?; emit.emit_block_start(true); }
            0x0B => emit.emit_block_end(),
            0x0C => { let depth = reader.read_u32_leb128()?; emit.emit_br(depth); }
            0x0D => { let depth = reader.read_u32_leb128()?; emit.emit_br_if(depth); }
            0x0F => emit.emit_return(),
            0x10 => { let _idx = reader.read_u32_leb128()?; emit.emit_nop(); } // call (stub for now)

            // === Locals ===
            0x20 => { let idx = reader.read_u32_leb128()?; emit.emit_local_get(idx); }
            0x21 => { let idx = reader.read_u32_leb128()?; emit.emit_local_set(idx); }
            0x22 => { let idx = reader.read_u32_leb128()?; emit.emit_local_tee(idx); }

            // === Memory ===
            0x28 => { let _a = reader.read_u32_leb128()?; let off = reader.read_u32_leb128()?; emit.emit_i32_load(off); }
            0x2A => { let _a = reader.read_u32_leb128()?; let off = reader.read_u32_leb128()?; emit.emit_i32_load(off); } // f32.load same encoding
            0x36 => { let _a = reader.read_u32_leb128()?; let off = reader.read_u32_leb128()?; emit.emit_i32_store(off); }
            0x38 => { let _a = reader.read_u32_leb128()?; let off = reader.read_u32_leb128()?; emit.emit_i32_store(off); } // f32.store same

            // === i32 Constants ===
            0x41 => { let val = reader.read_i32_leb128()?; emit.emit_i32_const(val); }

            // === f32 Constants ===
            0x43 => { let b = reader.read_bytes(4)?; let v = f32::from_le_bytes([b[0],b[1],b[2],b[3]]); emit.emit_i32_const(v.to_bits() as i32); }

            // === i32 Comparisons ===
            0x46 => emit.emit_i32_cmp(arm::COND_EQ),
            0x48 => emit.emit_i32_cmp(arm::COND_LT),
            0x4A => emit.emit_i32_cmp(arm::COND_GT),
            0x4C => emit.emit_i32_cmp(arm::COND_LE),

            // === i32 Arithmetic ===
            0x6A => emit.emit_i32_binop(BinOp::Add),
            0x6B => emit.emit_i32_binop(BinOp::Sub),
            0x6C => emit.emit_i32_binop(BinOp::Mul),
            0x71 => emit.emit_i32_binop(BinOp::And),
            0x72 => emit.emit_i32_binop(BinOp::Or),
            0x73 => emit.emit_i32_binop(BinOp::Xor),
            0x74 => emit.emit_i32_binop(BinOp::Shl),
            0x75 => emit.emit_i32_binop(BinOp::ShrS),

            // === f32 Arithmetic ===
            0x92 => emit.emit_f32_binop(FBinOp::Add),
            0x93 => emit.emit_f32_binop(FBinOp::Sub),
            0x94 => emit.emit_f32_binop(FBinOp::Mul),
            0x95 => emit.emit_f32_binop(FBinOp::Div),
            0x8B => emit.emit_f32_unary(FUnaryOp::Abs),
            0x8C => emit.emit_f32_unary(FUnaryOp::Neg),
            0x91 => emit.emit_f32_unary(FUnaryOp::Sqrt),

            // === Conversions ===
            0xA8 => emit.emit_f32_to_i32(),
            0xB2 => emit.emit_i32_to_f32(),

            _ => {} // Skip unknown opcodes
        }
    }

    emit.emit_epilogue();
    Ok(emit.finish())
}

// ── Binary operation types ──────────────────────────────────────

enum BinOp { Add, Sub, Mul, And, Or, Xor, Shl, ShrS }
enum FBinOp { Add, Sub, Mul, Div }
enum FUnaryOp { Abs, Neg, Sqrt }

// ── Code Emitter (aarch64) ──────────────────────────────────────

struct Emitter {
    code: Vec<u8>,
    blocks: Vec<BlockInfo>,
    fixups: Vec<Fixup>,
    local_frame_size: u32,
}

struct BlockInfo {
    start_offset: usize,
    is_loop: bool,
}

struct Fixup {
    /// Byte offset in code where the branch instruction is
    patch_offset: usize,
    /// Which block this branch targets (index into blocks stack at emit time)
    target_block_idx: usize,
    /// Is this a CBZ/CBNZ (19-bit imm) or B (26-bit imm)?
    is_conditional: bool,
}

// BodyReader (unchanged)
struct BodyReader<'a> { data: &'a [u8], pos: usize }

impl<'a> BodyReader<'a> {
    fn new(data: &'a [u8]) -> Self { BodyReader { data, pos: 0 } }
    fn remaining(&self) -> usize { self.data.len() - self.pos }
    fn read_byte(&mut self) -> Result<u8, SilverfirError> {
        if self.pos >= self.data.len() { return Err(SilverfirError::ParseError("EOF")); }
        let b = self.data[self.pos]; self.pos += 1; Ok(b)
    }
    fn read_bytes(&mut self, n: usize) -> Result<&'a [u8], SilverfirError> {
        if self.pos + n > self.data.len() { return Err(SilverfirError::ParseError("EOF")); }
        let s = &self.data[self.pos..self.pos+n]; self.pos += n; Ok(s)
    }
    fn read_u32_leb128(&mut self) -> Result<u32, SilverfirError> {
        let mut r: u32 = 0; let mut s = 0;
        loop { let b = self.read_byte()?; r |= ((b & 0x7F) as u32) << s;
            if b & 0x80 == 0 { break; } s += 7;
            if s >= 35 { return Err(SilverfirError::ParseError("LEB128")); } } Ok(r)
    }
    fn read_i32_leb128(&mut self) -> Result<i32, SilverfirError> {
        let mut r: i32 = 0; let mut s = 0; let mut b;
        loop { b = self.read_byte()?; r |= ((b & 0x7F) as i32) << s; s += 7;
            if b & 0x80 == 0 { break; }
            if s >= 35 { return Err(SilverfirError::ParseError("LEB128")); } }
        if s < 32 && (b & 0x40) != 0 { r |= !0 << s; } Ok(r)
    }
}

impl Emitter {
    fn new() -> Self {
        Emitter { code: Vec::with_capacity(4096), blocks: Vec::new(), fixups: Vec::new(), local_frame_size: 0 }
    }

    fn finish(self) -> Vec<u8> { self.code }
    fn pos(&self) -> usize { self.code.len() }

    fn emit(&mut self, inst: u32) {
        self.code.extend_from_slice(&inst.to_le_bytes());
    }

    fn emit_nop(&mut self) { self.emit(arm::nop()); }

    // ── Prologue / Epilogue ─────────────────────────────────────

    fn emit_prologue(&mut self, num_locals: u32) {
        // STP X29, X30, [SP, #-16]!
        self.emit(arm::stp_pre(arm::X29, arm::X30, arm::SP, -16));
        // MOV X29, SP
        self.emit(arm::add_imm64(arm::X29, arm::SP, 0));

        // Reserve space for locals on stack
        self.local_frame_size = num_locals * 8; // 8 bytes per local (aligned)
        if self.local_frame_size > 0 {
            // SUB SP, SP, #frame_size
            self.emit(arm::sub_imm64(arm::SP, arm::SP, self.local_frame_size as u16));
            // Zero-initialize locals
            for i in 0..num_locals {
                self.emit(arm::str_imm_w(arm::XZR as u8, arm::SP, (i * 8) as u16));
            }
        }
    }

    fn emit_epilogue(&mut self) {
        // Resolve all forward branch fixups
        self.resolve_fixups();

        // Restore SP from FP, then restore FP/LR
        self.emit(arm::add_imm64(arm::SP, arm::X29, 0)); // MOV SP, X29
        self.emit(arm::ldp_post(arm::X29, arm::X30, arm::SP, 16));
        self.emit(arm::ret());
    }

    fn emit_return(&mut self) {
        // Early return: pop result into W0 if stack has a value, then jump to epilogue
        self.emit(arm::pop_w(0)); // Pop return value into W0
        self.emit(arm::add_imm64(arm::SP, arm::X29, 0)); // MOV SP, X29
        self.emit(arm::ldp_post(arm::X29, arm::X30, arm::SP, 16));
        self.emit(arm::ret());
    }

    fn emit_trap(&mut self) {
        self.emit(arm::brk(0));
    }

    // ── Constants ───────────────────────────────────────────────

    fn emit_i32_const(&mut self, val: i32) {
        let insts = arm::mov_imm32(arm::W9, val as u32);
        for inst in &insts { self.emit(*inst); }
        self.emit(arm::push_w(arm::W9));
    }

    // ── i32 Binary Operations ───────────────────────────────────

    fn emit_i32_binop(&mut self, op: BinOp) {
        // Pop right operand into W10, left into W9
        self.emit(arm::pop_w(arm::W10));
        self.emit(arm::pop_w(arm::W9));

        // Compute W9 = W9 op W10
        let inst = match op {
            BinOp::Add  => arm::add_reg(arm::W9, arm::W9, arm::W10),
            BinOp::Sub  => arm::sub_reg(arm::W9, arm::W9, arm::W10),
            BinOp::Mul  => arm::mul_reg(arm::W9, arm::W9, arm::W10),
            BinOp::And  => arm::and_reg(arm::W9, arm::W9, arm::W10),
            BinOp::Or   => arm::orr_reg(arm::W9, arm::W9, arm::W10),
            BinOp::Xor  => arm::eor_reg(arm::W9, arm::W9, arm::W10),
            BinOp::Shl  => arm::lsl_reg(arm::W9, arm::W9, arm::W10),
            BinOp::ShrS => arm::asr_reg(arm::W9, arm::W9, arm::W10),
        };
        self.emit(inst);

        // Push result
        self.emit(arm::push_w(arm::W9));
    }

    // ── i32 Comparisons ─────────────────────────────────────────

    fn emit_i32_cmp(&mut self, cond: u8) {
        self.emit(arm::pop_w(arm::W10));
        self.emit(arm::pop_w(arm::W9));
        self.emit(arm::cmp_reg(arm::W9, arm::W10));
        self.emit(arm::cset(arm::W9, cond));
        self.emit(arm::push_w(arm::W9));
    }

    // ── f32 Binary Operations ───────────────────────────────────

    fn emit_f32_binop(&mut self, op: FBinOp) {
        // Pop right into W10, move to S1; pop left into W9, move to S0
        self.emit(arm::pop_w(arm::W10));
        self.emit(arm::fmov_sw(arm::S1, arm::W10));
        self.emit(arm::pop_w(arm::W9));
        self.emit(arm::fmov_sw(arm::S0, arm::W9));

        let inst = match op {
            FBinOp::Add => arm::fadd_s(arm::S0, arm::S0, arm::S1),
            FBinOp::Sub => arm::fsub_s(arm::S0, arm::S0, arm::S1),
            FBinOp::Mul => arm::fmul_s(arm::S0, arm::S0, arm::S1),
            FBinOp::Div => arm::fdiv_s(arm::S0, arm::S0, arm::S1),
        };
        self.emit(inst);

        // Move result back to integer register and push
        self.emit(arm::fmov_ws(arm::W9, arm::S0));
        self.emit(arm::push_w(arm::W9));
    }

    // ── f32 Unary Operations ────────────────────────────────────

    fn emit_f32_unary(&mut self, op: FUnaryOp) {
        self.emit(arm::pop_w(arm::W9));
        self.emit(arm::fmov_sw(arm::S0, arm::W9));

        let inst = match op {
            FUnaryOp::Abs  => arm::fabs_s(arm::S0, arm::S0),
            FUnaryOp::Neg  => arm::fneg_s(arm::S0, arm::S0),
            FUnaryOp::Sqrt => arm::fsqrt_s(arm::S0, arm::S0),
        };
        self.emit(inst);

        self.emit(arm::fmov_ws(arm::W9, arm::S0));
        self.emit(arm::push_w(arm::W9));
    }

    // ── Conversions ─────────────────────────────────────────────

    fn emit_f32_to_i32(&mut self) {
        self.emit(arm::pop_w(arm::W9));
        self.emit(arm::fmov_sw(arm::S0, arm::W9));
        self.emit(arm::fcvtzs_ws(arm::W9, arm::S0));
        self.emit(arm::push_w(arm::W9));
    }

    fn emit_i32_to_f32(&mut self) {
        self.emit(arm::pop_w(arm::W9));
        self.emit(arm::scvtf_sw(arm::S0, arm::W9));
        self.emit(arm::fmov_ws(arm::W9, arm::S0));
        self.emit(arm::push_w(arm::W9));
    }

    // ── Locals ──────────────────────────────────────────────────

    fn emit_local_get(&mut self, idx: u32) {
        // Locals are at [X29 - frame_size + idx*8]
        // But since we SUB'd SP by frame_size, locals are at [SP + idx*8] relative to start
        // Actually: locals are below FP, so: LDR W9, [X29, #-(frame_size - idx*8)]
        // Simpler: use SP-relative since SP = X29 - frame_size after prologue
        let offset = idx * 8;
        self.emit(arm::ldr_imm_w(arm::W9, arm::SP, offset as u16));
        self.emit(arm::push_w(arm::W9));
    }

    fn emit_local_set(&mut self, idx: u32) {
        self.emit(arm::pop_w(arm::W9));
        let offset = idx * 8;
        self.emit(arm::str_imm_w(arm::W9, arm::SP, offset as u16));
    }

    fn emit_local_tee(&mut self, idx: u32) {
        // Peek at top of stack (don't pop), store to local
        // LDR W9, [SP] (no post-increment)
        self.emit(arm::ldr_imm_w(arm::W9, arm::SP, 0));
        let offset = idx * 8;
        self.emit(arm::str_imm_w(arm::W9, arm::SP, offset as u16));
    }

    // ── Memory Operations ───────────────────────────────────────

    fn emit_i32_load(&mut self, offset: u32) {
        // Pop WASM address, add offset, add linear memory base (X0), load
        self.emit(arm::pop_w(arm::W11));
        if offset > 0 {
            let off_insts = arm::mov_imm32(arm::W10, offset);
            for i in &off_insts { self.emit(*i); }
            self.emit(arm::add_reg(arm::W11, arm::W11, arm::W10));
        }
        // LDR W9, [X0, X11] — X0 is linear memory base
        self.emit(arm::ldr_reg_w(arm::W9, arm::X0 as u8, arm::W11));
        self.emit(arm::push_w(arm::W9));
    }

    fn emit_i32_store(&mut self, offset: u32) {
        // Pop value, pop address
        self.emit(arm::pop_w(arm::W9));  // value
        self.emit(arm::pop_w(arm::W11)); // address
        if offset > 0 {
            let off_insts = arm::mov_imm32(arm::W10, offset);
            for i in &off_insts { self.emit(*i); }
            self.emit(arm::add_reg(arm::W11, arm::W11, arm::W10));
        }
        // STR W9, [X0, X11]
        self.emit(arm::str_reg_w(arm::W9, arm::X0 as u8, arm::W11));
    }

    // ── Control Flow with Branch Fixups ─────────────────────────

    fn emit_block_start(&mut self, is_loop: bool) {
        self.blocks.push(BlockInfo {
            start_offset: self.pos(),
            is_loop,
        });
    }

    fn emit_block_end(&mut self) {
        if let Some(block) = self.blocks.pop() {
            let end_offset = self.pos();
            // Resolve fixups targeting this block
            // (done lazily in resolve_fixups at epilogue)
            let _ = (block, end_offset);
        }
    }

    fn emit_br(&mut self, depth: u32) {
        let target_idx = if self.blocks.len() > depth as usize {
            self.blocks.len() - 1 - depth as usize
        } else {
            0
        };

        let block = &self.blocks[target_idx];

        if block.is_loop {
            // Loop: branch backward to block start
            let current = self.pos();
            let target = block.start_offset;
            let offset = (target as i32) - (current as i32);
            self.emit(arm::b(offset));
        } else {
            // Block: branch forward to block end (unknown yet → fixup)
            self.fixups.push(Fixup {
                patch_offset: self.pos(),
                target_block_idx: target_idx,
                is_conditional: false,
            });
            self.emit(arm::b(0)); // Placeholder, will be patched
        }
    }

    fn emit_br_if(&mut self, depth: u32) {
        // Pop condition
        self.emit(arm::pop_w(arm::W9));

        let target_idx = if self.blocks.len() > depth as usize {
            self.blocks.len() - 1 - depth as usize
        } else {
            0
        };

        let block = &self.blocks[target_idx];

        if block.is_loop {
            let current = self.pos();
            let target = block.start_offset;
            let offset = (target as i32) - (current as i32);
            self.emit(arm::cbnz_w(arm::W9, offset));
        } else {
            self.fixups.push(Fixup {
                patch_offset: self.pos(),
                target_block_idx: target_idx,
                is_conditional: true,
            });
            self.emit(arm::cbnz_w(arm::W9, 0)); // Placeholder
        }
    }

    /// Resolve all forward branch fixups.
    /// Called at the end of compilation when all block end offsets are known.
    fn resolve_fixups(&mut self) {
        // At this point, all blocks are closed. The end offset for each block
        // is approximated as the current position (end of function).
        let end = self.pos();

        for fixup in &self.fixups {
            let target_offset = end; // All forward branches go to function end for now
            let branch_offset = (target_offset as i32) - (fixup.patch_offset as i32);

            let patched = if fixup.is_conditional {
                arm::cbnz_w(arm::W9, branch_offset)
            } else {
                arm::b(branch_offset)
            };

            // Patch the instruction in-place
            let bytes = patched.to_le_bytes();
            self.code[fixup.patch_offset] = bytes[0];
            self.code[fixup.patch_offset + 1] = bytes[1];
            self.code[fixup.patch_offset + 2] = bytes[2];
            self.code[fixup.patch_offset + 3] = bytes[3];
        }
    }
}
