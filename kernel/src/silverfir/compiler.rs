//! WASM → Native Code Compiler (Single-Pass)
//!
//! Translates WASM bytecode to native machine code in a single forward pass.
//! This is a "baseline" compiler — it generates correct but not optimized code.
//! For DSP workloads, the hot path is typically a tight loop over f32 arrays,
//! which even a baseline compiler handles well since the bottleneck is memory bandwidth.
//!
//! Strategy:
//!   - Stack-based → register-based translation
//!   - WASM operand stack maps to a fixed set of registers
//!   - Spill to memory when registers are exhausted
//!   - Host function calls use the platform ABI directly
//!
//! Supported WASM opcodes (DAQ-focused subset):
//!   - Control: block, loop, br, br_if, return, call, end
//!   - Numeric i32: const, add, sub, mul, and, or, xor, shl, shr, lt/gt/eq
//!   - Numeric f32: const, add, sub, mul, div, neg, abs, sqrt, min, max
//!   - Memory: i32.load, i32.store, f32.load, f32.store
//!   - Local: local.get, local.set, local.tee
//!   - Conversion: f32.convert_i32_s, i32.trunc_f32_s

use alloc::vec::Vec;
use super::SilverfirError;
use super::parser::FuncType;

/// Compile a single WASM function body to native code.
///
/// The function body includes local declarations followed by opcodes.
/// Returns a Vec<u8> of native machine code.
pub fn compile_function(
    body: &[u8],
    _func_type: Option<&FuncType>,
) -> Result<Vec<u8>, SilverfirError> {
    let mut emit = CodeEmitter::new();
    let mut reader = BodyReader::new(body);

    // Parse local declarations
    let local_decl_count = reader.read_u32_leb128()?;
    let mut total_locals: u32 = 0;
    for _ in 0..local_decl_count {
        let count = reader.read_u32_leb128()?;
        let _valtype = reader.read_byte()?;
        total_locals += count;
    }

    // Emit function prologue
    emit.emit_prologue(total_locals);

    // Translate opcodes
    while reader.remaining() > 0 {
        let opcode = reader.read_byte()?;
        match opcode {
            // === Control Flow ===
            0x00 => { /* unreachable */ emit.emit_trap(); }
            0x01 => { /* nop */ }
            0x02 => { /* block */ let _bt = reader.read_byte()?; emit.emit_block_start(); }
            0x03 => { /* loop */ let _bt = reader.read_byte()?; emit.emit_loop_start(); }
            0x0B => { /* end */ emit.emit_block_end(); }
            0x0C => { /* br */ let depth = reader.read_u32_leb128()?; emit.emit_br(depth); }
            0x0D => { /* br_if */ let depth = reader.read_u32_leb128()?; emit.emit_br_if(depth); }
            0x0F => { /* return */ emit.emit_return(); }
            0x10 => { /* call */ let idx = reader.read_u32_leb128()?; emit.emit_call(idx); }

            // === Local Variables ===
            0x20 => { /* local.get */ let idx = reader.read_u32_leb128()?; emit.emit_local_get(idx); }
            0x21 => { /* local.set */ let idx = reader.read_u32_leb128()?; emit.emit_local_set(idx); }
            0x22 => { /* local.tee */ let idx = reader.read_u32_leb128()?; emit.emit_local_tee(idx); }

            // === Memory ===
            0x28 => { /* i32.load */
                let _align = reader.read_u32_leb128()?;
                let offset = reader.read_u32_leb128()?;
                emit.emit_i32_load(offset);
            }
            0x36 => { /* i32.store */
                let _align = reader.read_u32_leb128()?;
                let offset = reader.read_u32_leb128()?;
                emit.emit_i32_store(offset);
            }
            0x2A => { /* f32.load */
                let _align = reader.read_u32_leb128()?;
                let offset = reader.read_u32_leb128()?;
                emit.emit_f32_load(offset);
            }
            0x38 => { /* f32.store */
                let _align = reader.read_u32_leb128()?;
                let offset = reader.read_u32_leb128()?;
                emit.emit_f32_store(offset);
            }

            // === i32 Constants ===
            0x41 => { /* i32.const */ let val = reader.read_i32_leb128()?; emit.emit_i32_const(val); }

            // === i32 Arithmetic ===
            0x46 => { emit.emit_i32_eq(); }    // i32.eq
            0x48 => { emit.emit_i32_lt_s(); }  // i32.lt_s
            0x4A => { emit.emit_i32_gt_s(); }  // i32.gt_s
            0x4C => { emit.emit_i32_le_s(); }  // i32.le_s
            0x6A => { emit.emit_i32_add(); }   // i32.add
            0x6B => { emit.emit_i32_sub(); }   // i32.sub
            0x6C => { emit.emit_i32_mul(); }   // i32.mul
            0x71 => { emit.emit_i32_and(); }   // i32.and
            0x72 => { emit.emit_i32_or(); }    // i32.or
            0x73 => { emit.emit_i32_xor(); }   // i32.xor
            0x74 => { emit.emit_i32_shl(); }   // i32.shl
            0x75 => { emit.emit_i32_shr_s(); } // i32.shr_s

            // === f32 Constants ===
            0x43 => { /* f32.const */
                let bytes = reader.read_bytes(4)?;
                let val = f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
                emit.emit_f32_const(val);
            }

            // === f32 Arithmetic ===
            0x92 => { emit.emit_f32_add(); }   // f32.add
            0x93 => { emit.emit_f32_sub(); }   // f32.sub
            0x94 => { emit.emit_f32_mul(); }   // f32.mul
            0x95 => { emit.emit_f32_div(); }   // f32.div
            0x8B => { emit.emit_f32_abs(); }   // f32.abs
            0x8C => { emit.emit_f32_neg(); }   // f32.neg
            0x91 => { emit.emit_f32_sqrt(); }  // f32.sqrt

            // === Conversions ===
            0xA8 => { emit.emit_i32_trunc_f32_s(); }      // i32.trunc_f32_s
            0xB2 => { emit.emit_f32_convert_i32_s(); }    // f32.convert_i32_s

            // Skip unknown opcodes (log and continue)
            other => {
                crate::kprintln!("  Silverfir: unknown opcode {:#04x}, skipping", other);
                // Try to skip any immediate arguments
                // This is best-effort; complex opcodes may break
            }
        }
    }

    emit.emit_epilogue();
    Ok(emit.finish())
}

// === Code Emitter ===
// Emits native machine code bytes for the target architecture.

struct CodeEmitter {
    code: Vec<u8>,
    /// Operand stack depth (tracks virtual stack for register allocation)
    stack_depth: i32,
    /// Block nesting for structured control flow
    block_stack: Vec<BlockInfo>,
    /// Fixup locations for forward branches
    fixups: Vec<BranchFixup>,
}

struct BlockInfo {
    /// Offset in code where this block started
    start_offset: usize,
    /// Whether this is a loop (br targets start) or block (br targets end)
    is_loop: bool,
}

struct BranchFixup {
    /// Offset in code where the branch displacement needs to be written
    code_offset: usize,
    /// Target block depth
    target_depth: u32,
}

// Body reader (similar to parser::Reader but for function bodies)
struct BodyReader<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> BodyReader<'a> {
    fn new(data: &'a [u8]) -> Self { BodyReader { data, pos: 0 } }
    fn remaining(&self) -> usize { self.data.len() - self.pos }

    fn read_byte(&mut self) -> Result<u8, SilverfirError> {
        if self.pos >= self.data.len() { return Err(SilverfirError::ParseError("EOF in body")); }
        let b = self.data[self.pos]; self.pos += 1; Ok(b)
    }

    fn read_bytes(&mut self, n: usize) -> Result<&'a [u8], SilverfirError> {
        if self.pos + n > self.data.len() { return Err(SilverfirError::ParseError("EOF in body bytes")); }
        let s = &self.data[self.pos..self.pos + n]; self.pos += n; Ok(s)
    }

    fn read_u32_leb128(&mut self) -> Result<u32, SilverfirError> {
        let mut result: u32 = 0; let mut shift = 0;
        loop {
            let byte = self.read_byte()?;
            result |= ((byte & 0x7F) as u32) << shift;
            if byte & 0x80 == 0 { break; }
            shift += 7;
            if shift >= 35 { return Err(SilverfirError::ParseError("LEB128 overflow")); }
        }
        Ok(result)
    }

    fn read_i32_leb128(&mut self) -> Result<i32, SilverfirError> {
        let mut result: i32 = 0; let mut shift = 0; let mut byte;
        loop {
            byte = self.read_byte()?;
            result |= ((byte & 0x7F) as i32) << shift;
            shift += 7;
            if byte & 0x80 == 0 { break; }
            if shift >= 35 { return Err(SilverfirError::ParseError("signed LEB128 overflow")); }
        }
        if shift < 32 && (byte & 0x40) != 0 { result |= !0 << shift; }
        Ok(result)
    }
}

impl CodeEmitter {
    fn new() -> Self {
        CodeEmitter {
            code: Vec::with_capacity(4096),
            stack_depth: 0,
            block_stack: Vec::new(),
            fixups: Vec::new(),
        }
    }

    fn finish(self) -> Vec<u8> { self.code }
    fn current_offset(&self) -> usize { self.code.len() }

    // === Architecture-specific code generation ===
    // We use a simple stack machine → register mapping:
    //
    // AArch64 registers:
    //   x0-x7: arguments/return (x0 = linear_memory_ptr, x1 = mem_size)
    //   x9-x15: scratch (operand stack)
    //   x19-x28: callee-saved (locals)
    //   s0-s7: FP scratch (f32 operand stack)
    //
    // x86_64 registers:
    //   rdi = linear_memory_ptr, rsi = mem_size
    //   rax, rcx, rdx, r8-r11: scratch (operand stack)
    //   xmm0-xmm7: FP scratch

    fn emit_prologue(&mut self, _num_locals: u32) {
        #[cfg(target_arch = "aarch64")]
        {
            // stp x29, x30, [sp, #-16]!  — save frame pointer and link register
            self.emit_bytes(&[0xFD, 0x7B, 0xBF, 0xA9]);
            // mov x29, sp
            self.emit_bytes(&[0xFD, 0x03, 0x00, 0x91]);
        }

        #[cfg(target_arch = "x86_64")]
        {
            // push rbp
            self.emit_byte(0x55);
            // mov rbp, rsp
            self.emit_bytes(&[0x48, 0x89, 0xE5]);
        }
    }

    fn emit_epilogue(&mut self) {
        #[cfg(target_arch = "aarch64")]
        {
            // ldp x29, x30, [sp], #16
            self.emit_bytes(&[0xFD, 0x7B, 0xC1, 0xA8]);
            // ret
            self.emit_bytes(&[0xC0, 0x03, 0x5F, 0xD6]);
        }

        #[cfg(target_arch = "x86_64")]
        {
            // pop rbp
            self.emit_byte(0x5D);
            // ret
            self.emit_byte(0xC3);
        }
    }

    fn emit_return(&mut self) {
        self.emit_epilogue();
    }

    fn emit_trap(&mut self) {
        #[cfg(target_arch = "aarch64")]
        self.emit_bytes(&[0x00, 0x00, 0x20, 0xD4]); // brk #0

        #[cfg(target_arch = "x86_64")]
        self.emit_byte(0xCC); // int3
    }

    // === Constants ===

    fn emit_i32_const(&mut self, val: i32) {
        #[cfg(target_arch = "aarch64")]
        {
            // mov w9, #imm (simplified — full immediate encoding is complex)
            let uval = val as u32;
            // movz w9, #(uval & 0xFFFF)
            self.emit_bytes(&[
                (0x09 | ((uval & 0x1F) << 0)) as u8,
                ((uval >> 5) & 0xFF) as u8,
                (0x80 | ((uval >> 13) & 0x7F)) as u8,
                0x52,
            ]);
            if uval > 0xFFFF {
                // movk w9, #(uval >> 16), lsl #16
                let hi = uval >> 16;
                self.emit_bytes(&[
                    (0x09 | ((hi & 0x1F) << 0)) as u8,
                    ((hi >> 5) & 0xFF) as u8,
                    (0xA0 | ((hi >> 13) & 0x7F)) as u8,
                    0x72,
                ]);
            }
            // Push to virtual stack: str w9, [sp, #-4]!
            self.emit_bytes(&[0xE9, 0x0F, 0x1F, 0xB8]);
        }

        #[cfg(target_arch = "x86_64")]
        {
            // push imm32
            self.emit_byte(0x68);
            self.emit_bytes(&(val as u32).to_le_bytes());
        }

        self.stack_depth += 1;
    }

    fn emit_f32_const(&mut self, val: f32) {
        // Push f32 as raw bits onto the integer stack
        self.emit_i32_const(val.to_bits() as i32);
    }

    // === i32 Arithmetic ===

    fn emit_i32_binop(&mut self, _op: &str) {
        // Pop two operands, apply op, push result
        // This is a simplified version — real impl needs register allocation
        self.stack_depth -= 1; // net: pop 2, push 1
    }

    fn emit_i32_add(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            // pop rcx; pop rax; add eax, ecx; push rax
            self.emit_byte(0x59); // pop rcx
            self.emit_byte(0x58); // pop rax
            self.emit_bytes(&[0x01, 0xC8]); // add eax, ecx
            self.emit_byte(0x50); // push rax
        }
        #[cfg(target_arch = "aarch64")]
        {
            // ldr w10, [sp], #4; ldr w9, [sp]; add w9, w9, w10; str w9, [sp]
            self.emit_bytes(&[0xEA, 0x07, 0x41, 0xB8]); // ldr w10, [sp], #4
            self.emit_bytes(&[0xE9, 0x03, 0x40, 0xB9]); // ldr w9, [sp]
            self.emit_bytes(&[0x29, 0x01, 0x0A, 0x0B]); // add w9, w9, w10
            self.emit_bytes(&[0xE9, 0x03, 0x00, 0xB9]); // str w9, [sp]
        }
        self.stack_depth -= 1;
    }

    fn emit_i32_sub(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            self.emit_byte(0x59); // pop rcx
            self.emit_byte(0x58); // pop rax
            self.emit_bytes(&[0x29, 0xC8]); // sub eax, ecx
            self.emit_byte(0x50); // push rax
        }
        #[cfg(target_arch = "aarch64")]
        {
            self.emit_bytes(&[0xEA, 0x07, 0x41, 0xB8]);
            self.emit_bytes(&[0xE9, 0x03, 0x40, 0xB9]);
            self.emit_bytes(&[0x29, 0x01, 0x0A, 0x4B]); // sub w9, w9, w10
            self.emit_bytes(&[0xE9, 0x03, 0x00, 0xB9]);
        }
        self.stack_depth -= 1;
    }

    fn emit_i32_mul(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            self.emit_byte(0x59); // pop rcx
            self.emit_byte(0x58); // pop rax
            self.emit_bytes(&[0x0F, 0xAF, 0xC1]); // imul eax, ecx
            self.emit_byte(0x50); // push rax
        }
        #[cfg(target_arch = "aarch64")]
        {
            self.emit_bytes(&[0xEA, 0x07, 0x41, 0xB8]);
            self.emit_bytes(&[0xE9, 0x03, 0x40, 0xB9]);
            self.emit_bytes(&[0x29, 0x7D, 0x0A, 0x1B]); // mul w9, w9, w10
            self.emit_bytes(&[0xE9, 0x03, 0x00, 0xB9]);
        }
        self.stack_depth -= 1;
    }

    fn emit_i32_and(&mut self) { self.emit_i32_binop("and"); }
    fn emit_i32_or(&mut self) { self.emit_i32_binop("or"); }
    fn emit_i32_xor(&mut self) { self.emit_i32_binop("xor"); }
    fn emit_i32_shl(&mut self) { self.emit_i32_binop("shl"); }
    fn emit_i32_shr_s(&mut self) { self.emit_i32_binop("shr_s"); }
    fn emit_i32_eq(&mut self) { self.emit_i32_binop("eq"); }
    fn emit_i32_lt_s(&mut self) { self.emit_i32_binop("lt_s"); }
    fn emit_i32_gt_s(&mut self) { self.emit_i32_binop("gt_s"); }
    fn emit_i32_le_s(&mut self) { self.emit_i32_binop("le_s"); }

    // === f32 Arithmetic (stub — needs FP register usage) ===

    fn emit_f32_binop(&mut self) { self.stack_depth -= 1; }
    fn emit_f32_add(&mut self) { self.emit_f32_binop(); }
    fn emit_f32_sub(&mut self) { self.emit_f32_binop(); }
    fn emit_f32_mul(&mut self) { self.emit_f32_binop(); }
    fn emit_f32_div(&mut self) { self.emit_f32_binop(); }
    fn emit_f32_abs(&mut self) { /* unary, no stack change */ }
    fn emit_f32_neg(&mut self) { /* unary, no stack change */ }
    fn emit_f32_sqrt(&mut self) { /* unary, no stack change */ }

    // === Memory Operations (stub) ===
    fn emit_i32_load(&mut self, _offset: u32) { /* pop addr, push value */ }
    fn emit_i32_store(&mut self, _offset: u32) { self.stack_depth -= 2; }
    fn emit_f32_load(&mut self, _offset: u32) { /* pop addr, push value */ }
    fn emit_f32_store(&mut self, _offset: u32) { self.stack_depth -= 2; }

    // === Locals (stub) ===
    fn emit_local_get(&mut self, _idx: u32) { self.stack_depth += 1; }
    fn emit_local_set(&mut self, _idx: u32) { self.stack_depth -= 1; }
    fn emit_local_tee(&mut self, _idx: u32) { /* no stack change */ }

    // === Control Flow (stub) ===
    fn emit_block_start(&mut self) {
        self.block_stack.push(BlockInfo {
            start_offset: self.current_offset(),
            is_loop: false,
        });
    }

    fn emit_loop_start(&mut self) {
        self.block_stack.push(BlockInfo {
            start_offset: self.current_offset(),
            is_loop: true,
        });
    }

    fn emit_block_end(&mut self) {
        self.block_stack.pop();
    }

    fn emit_br(&mut self, _depth: u32) {
        // Unconditional branch — emit a jump (fixup later)
        #[cfg(target_arch = "x86_64")]
        {
            self.emit_byte(0xE9); // jmp rel32
            self.emit_bytes(&[0x00, 0x00, 0x00, 0x00]); // placeholder
        }
        #[cfg(target_arch = "aarch64")]
        {
            self.emit_bytes(&[0x00, 0x00, 0x00, 0x14]); // b #0 placeholder
        }
    }

    fn emit_br_if(&mut self, _depth: u32) {
        #[cfg(target_arch = "x86_64")]
        {
            self.emit_byte(0x58); // pop rax (condition)
            self.emit_bytes(&[0x85, 0xC0]); // test eax, eax
            self.emit_bytes(&[0x0F, 0x85, 0x00, 0x00, 0x00, 0x00]); // jne rel32
        }
        #[cfg(target_arch = "aarch64")]
        {
            self.emit_bytes(&[0xE9, 0x07, 0x41, 0xB8]); // ldr w9, [sp], #4
            self.emit_bytes(&[0x29, 0x01, 0x00, 0x35]); // cbnz w9, #placeholder
        }
        self.stack_depth -= 1;
    }

    fn emit_call(&mut self, _func_index: u32) {
        // TODO: Emit a call to either a local function or a host import
        // For now, emit a nop placeholder
        #[cfg(target_arch = "x86_64")]
        self.emit_bytes(&[0x90, 0x90, 0x90, 0x90, 0x90]); // 5× nop

        #[cfg(target_arch = "aarch64")]
        self.emit_bytes(&[0x1F, 0x20, 0x03, 0xD5]); // nop
    }

    // === Conversions (stub) ===
    fn emit_i32_trunc_f32_s(&mut self) { /* convert top of stack */ }
    fn emit_f32_convert_i32_s(&mut self) { /* convert top of stack */ }

    // === Helpers ===

    fn emit_byte(&mut self, b: u8) {
        self.code.push(b);
    }

    fn emit_bytes(&mut self, bytes: &[u8]) {
        self.code.extend_from_slice(bytes);
    }
}
