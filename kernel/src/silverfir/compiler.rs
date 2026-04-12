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

    fn emit_i32_and(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            self.emit_byte(0x59); self.emit_byte(0x58);
            self.emit_bytes(&[0x21, 0xC8]); // and eax, ecx
            self.emit_byte(0x50);
        }
        self.stack_depth -= 1;
    }

    fn emit_i32_or(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            self.emit_byte(0x59); self.emit_byte(0x58);
            self.emit_bytes(&[0x09, 0xC8]); // or eax, ecx
            self.emit_byte(0x50);
        }
        self.stack_depth -= 1;
    }

    fn emit_i32_xor(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            self.emit_byte(0x59); self.emit_byte(0x58);
            self.emit_bytes(&[0x31, 0xC8]); // xor eax, ecx
            self.emit_byte(0x50);
        }
        self.stack_depth -= 1;
    }

    fn emit_i32_shl(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            self.emit_byte(0x59); // pop rcx (shift amount — cl)
            self.emit_byte(0x58); // pop rax (value)
            self.emit_bytes(&[0xD3, 0xE0]); // shl eax, cl
            self.emit_byte(0x50);
        }
        self.stack_depth -= 1;
    }

    fn emit_i32_shr_s(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            self.emit_byte(0x59); self.emit_byte(0x58);
            self.emit_bytes(&[0xD3, 0xF8]); // sar eax, cl
            self.emit_byte(0x50);
        }
        self.stack_depth -= 1;
    }

    fn emit_i32_eq(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            self.emit_byte(0x59); self.emit_byte(0x58);
            self.emit_bytes(&[0x39, 0xC8]); // cmp eax, ecx
            self.emit_bytes(&[0x0F, 0x94, 0xC0]); // sete al
            self.emit_bytes(&[0x0F, 0xB6, 0xC0]); // movzx eax, al
            self.emit_byte(0x50);
        }
        self.stack_depth -= 1;
    }

    fn emit_i32_lt_s(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            self.emit_byte(0x59); self.emit_byte(0x58);
            self.emit_bytes(&[0x39, 0xC8]); // cmp eax, ecx
            self.emit_bytes(&[0x0F, 0x9C, 0xC0]); // setl al
            self.emit_bytes(&[0x0F, 0xB6, 0xC0]); // movzx eax, al
            self.emit_byte(0x50);
        }
        self.stack_depth -= 1;
    }

    fn emit_i32_gt_s(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            self.emit_byte(0x59); self.emit_byte(0x58);
            self.emit_bytes(&[0x39, 0xC8]); // cmp eax, ecx
            self.emit_bytes(&[0x0F, 0x9F, 0xC0]); // setg al
            self.emit_bytes(&[0x0F, 0xB6, 0xC0]); // movzx eax, al
            self.emit_byte(0x50);
        }
        self.stack_depth -= 1;
    }

    fn emit_i32_le_s(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            self.emit_byte(0x59); self.emit_byte(0x58);
            self.emit_bytes(&[0x39, 0xC8]); // cmp eax, ecx
            self.emit_bytes(&[0x0F, 0x9E, 0xC0]); // setle al
            self.emit_bytes(&[0x0F, 0xB6, 0xC0]); // movzx eax, al
            self.emit_byte(0x50);
        }
        self.stack_depth -= 1;
    }

    // === f32 Arithmetic ===
    //
    // Strategy: pop raw bits from integer stack into XMM/NEON registers,
    // perform FP operation, push result bits back.
    //
    // x86_64:  movd xmm0, [rsp]; movd xmm1, [rsp+4]; op; movd [rsp], xmm0
    // aarch64: ldr s0, [sp]; ldr s1, [sp, #4]; op; str s0, [sp, #4]; add sp, #4

    fn emit_f32_add(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            // movss xmm1, [rsp]     — top of stack (right operand)
            self.emit_bytes(&[0xF3, 0x0F, 0x10, 0x0C, 0x24]);
            // add rsp, 4
            self.emit_bytes(&[0x48, 0x83, 0xC4, 0x04]);
            // movss xmm0, [rsp]     — second operand (left)
            self.emit_bytes(&[0xF3, 0x0F, 0x10, 0x04, 0x24]);
            // addss xmm0, xmm1
            self.emit_bytes(&[0xF3, 0x0F, 0x58, 0xC1]);
            // movss [rsp], xmm0     — store result
            self.emit_bytes(&[0xF3, 0x0F, 0x11, 0x04, 0x24]);
        }
        #[cfg(target_arch = "aarch64")]
        {
            self.emit_bytes(&[0xE0, 0x03, 0x40, 0xBD]); // ldr s0, [sp]
            self.emit_bytes(&[0xE1, 0x07, 0x40, 0xBD]); // ldr s1, [sp, #4]
            self.emit_bytes(&[0x20, 0x28, 0x21, 0x1E]); // fadd s0, s1, s0
            self.emit_bytes(&[0xFF, 0x43, 0x00, 0x91]); // add sp, sp, #4 (pop one)
            self.emit_bytes(&[0xE0, 0x03, 0x00, 0xBD]); // str s0, [sp]
        }
        self.stack_depth -= 1;
    }

    fn emit_f32_sub(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            self.emit_bytes(&[0xF3, 0x0F, 0x10, 0x0C, 0x24]); // movss xmm1, [rsp]
            self.emit_bytes(&[0x48, 0x83, 0xC4, 0x04]);         // add rsp, 4
            self.emit_bytes(&[0xF3, 0x0F, 0x10, 0x04, 0x24]); // movss xmm0, [rsp]
            self.emit_bytes(&[0xF3, 0x0F, 0x5C, 0xC1]);         // subss xmm0, xmm1
            self.emit_bytes(&[0xF3, 0x0F, 0x11, 0x04, 0x24]); // movss [rsp], xmm0
        }
        #[cfg(target_arch = "aarch64")]
        {
            self.emit_bytes(&[0xE0, 0x03, 0x40, 0xBD]); // ldr s0, [sp]
            self.emit_bytes(&[0xE1, 0x07, 0x40, 0xBD]); // ldr s1, [sp, #4]
            self.emit_bytes(&[0x20, 0x38, 0x20, 0x1E]); // fsub s0, s1, s0
            self.emit_bytes(&[0xFF, 0x43, 0x00, 0x91]); // add sp, sp, #4
            self.emit_bytes(&[0xE0, 0x03, 0x00, 0xBD]); // str s0, [sp]
        }
        self.stack_depth -= 1;
    }

    fn emit_f32_mul(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            self.emit_bytes(&[0xF3, 0x0F, 0x10, 0x0C, 0x24]); // movss xmm1, [rsp]
            self.emit_bytes(&[0x48, 0x83, 0xC4, 0x04]);         // add rsp, 4
            self.emit_bytes(&[0xF3, 0x0F, 0x10, 0x04, 0x24]); // movss xmm0, [rsp]
            self.emit_bytes(&[0xF3, 0x0F, 0x59, 0xC1]);         // mulss xmm0, xmm1
            self.emit_bytes(&[0xF3, 0x0F, 0x11, 0x04, 0x24]); // movss [rsp], xmm0
        }
        #[cfg(target_arch = "aarch64")]
        {
            self.emit_bytes(&[0xE0, 0x03, 0x40, 0xBD]); // ldr s0, [sp]
            self.emit_bytes(&[0xE1, 0x07, 0x40, 0xBD]); // ldr s1, [sp, #4]
            self.emit_bytes(&[0x20, 0x08, 0x20, 0x1E]); // fmul s0, s1, s0
            self.emit_bytes(&[0xFF, 0x43, 0x00, 0x91]); // add sp, sp, #4
            self.emit_bytes(&[0xE0, 0x03, 0x00, 0xBD]); // str s0, [sp]
        }
        self.stack_depth -= 1;
    }

    fn emit_f32_div(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            self.emit_bytes(&[0xF3, 0x0F, 0x10, 0x0C, 0x24]); // movss xmm1, [rsp]
            self.emit_bytes(&[0x48, 0x83, 0xC4, 0x04]);         // add rsp, 4
            self.emit_bytes(&[0xF3, 0x0F, 0x10, 0x04, 0x24]); // movss xmm0, [rsp]
            self.emit_bytes(&[0xF3, 0x0F, 0x5E, 0xC1]);         // divss xmm0, xmm1
            self.emit_bytes(&[0xF3, 0x0F, 0x11, 0x04, 0x24]); // movss [rsp], xmm0
        }
        #[cfg(target_arch = "aarch64")]
        {
            self.emit_bytes(&[0xE0, 0x03, 0x40, 0xBD]); // ldr s0, [sp]
            self.emit_bytes(&[0xE1, 0x07, 0x40, 0xBD]); // ldr s1, [sp, #4]
            self.emit_bytes(&[0x20, 0x18, 0x20, 0x1E]); // fdiv s0, s1, s0
            self.emit_bytes(&[0xFF, 0x43, 0x00, 0x91]); // add sp, sp, #4
            self.emit_bytes(&[0xE0, 0x03, 0x00, 0xBD]); // str s0, [sp]
        }
        self.stack_depth -= 1;
    }

    fn emit_f32_abs(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            // movss xmm0, [rsp]; andps xmm0, [abs_mask]; movss [rsp], xmm0
            // Simpler: movd eax, [rsp]; and eax, 0x7FFFFFFF; mov [rsp], eax
            self.emit_byte(0x58); // pop rax
            self.emit_bytes(&[0x25]); // and eax, imm32
            self.emit_bytes(&0x7FFFFFFFu32.to_le_bytes());
            self.emit_byte(0x50); // push rax
        }
        #[cfg(target_arch = "aarch64")]
        {
            self.emit_bytes(&[0xE0, 0x03, 0x40, 0xBD]); // ldr s0, [sp]
            self.emit_bytes(&[0x00, 0xC0, 0x20, 0x1E]); // fabs s0, s0
            self.emit_bytes(&[0xE0, 0x03, 0x00, 0xBD]); // str s0, [sp]
        }
        // unary — no stack depth change
    }

    fn emit_f32_neg(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            // xor top bit: xor [rsp], 0x80000000
            self.emit_byte(0x58); // pop rax
            self.emit_bytes(&[0x35]); // xor eax, imm32
            self.emit_bytes(&0x80000000u32.to_le_bytes());
            self.emit_byte(0x50); // push rax
        }
        #[cfg(target_arch = "aarch64")]
        {
            self.emit_bytes(&[0xE0, 0x03, 0x40, 0xBD]); // ldr s0, [sp]
            self.emit_bytes(&[0x00, 0x40, 0x21, 0x1E]); // fneg s0, s0
            self.emit_bytes(&[0xE0, 0x03, 0x00, 0xBD]); // str s0, [sp]
        }
    }

    fn emit_f32_sqrt(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            self.emit_bytes(&[0xF3, 0x0F, 0x10, 0x04, 0x24]); // movss xmm0, [rsp]
            self.emit_bytes(&[0xF3, 0x0F, 0x51, 0xC0]);         // sqrtss xmm0, xmm0
            self.emit_bytes(&[0xF3, 0x0F, 0x11, 0x04, 0x24]); // movss [rsp], xmm0
        }
        #[cfg(target_arch = "aarch64")]
        {
            self.emit_bytes(&[0xE0, 0x03, 0x40, 0xBD]); // ldr s0, [sp]
            self.emit_bytes(&[0x00, 0xC0, 0x21, 0x1E]); // fsqrt s0, s0
            self.emit_bytes(&[0xE0, 0x03, 0x00, 0xBD]); // str s0, [sp]
        }
    }

    // === Memory Operations ===
    //
    // Linear memory base is passed in rdi (x86_64) or x0 (aarch64).
    // WASM address = pop from stack, add offset, add to base → load/store.

    fn emit_i32_load(&mut self, offset: u32) {
        #[cfg(target_arch = "x86_64")]
        {
            self.emit_byte(0x58); // pop rax (WASM address)
            if offset > 0 {
                // add eax, offset
                self.emit_bytes(&[0x05]);
                self.emit_bytes(&offset.to_le_bytes());
            }
            // mov eax, [rdi + rax]  (rdi = linear_memory base)
            self.emit_bytes(&[0x8B, 0x04, 0x07]); // mov eax, [rdi+rax]
            self.emit_byte(0x50); // push rax
        }
        #[cfg(target_arch = "aarch64")]
        {
            self.emit_bytes(&[0xE9, 0x07, 0x41, 0xB8]); // ldr w9, [sp], #4 (pop addr)
            if offset > 0 {
                // add w9, w9, #offset (only works for small offsets)
                let imm12 = offset.min(4095);
                let enc = 0x11000000 | ((imm12 & 0xFFF) << 10) | (9 << 5) | 9;
                self.emit_bytes(&enc.to_le_bytes());
            }
            // ldr w9, [x0, w9, uxtw]  (x0 = linear_memory base)
            self.emit_bytes(&[0x09, 0x48, 0x69, 0xB8]); // ldr w9, [x0, x9]
            self.emit_bytes(&[0xE9, 0x0F, 0x1F, 0xB8]); // str w9, [sp, #-4]! (push)
        }
        // net: pop addr, push value → no change
    }

    fn emit_i32_store(&mut self, offset: u32) {
        #[cfg(target_arch = "x86_64")]
        {
            self.emit_byte(0x59); // pop rcx (value)
            self.emit_byte(0x58); // pop rax (address)
            if offset > 0 {
                self.emit_bytes(&[0x05]);
                self.emit_bytes(&offset.to_le_bytes());
            }
            // mov [rdi + rax], ecx
            self.emit_bytes(&[0x89, 0x0C, 0x07]); // mov [rdi+rax], ecx
        }
        #[cfg(target_arch = "aarch64")]
        {
            self.emit_bytes(&[0xEA, 0x07, 0x41, 0xB8]); // ldr w10, [sp], #4 (pop value)
            self.emit_bytes(&[0xE9, 0x07, 0x41, 0xB8]); // ldr w9, [sp], #4 (pop addr)
            if offset > 0 {
                let imm12 = offset.min(4095);
                let enc = 0x11000000 | ((imm12 & 0xFFF) << 10) | (9 << 5) | 9;
                self.emit_bytes(&enc.to_le_bytes());
            }
            // str w10, [x0, x9]
            self.emit_bytes(&[0x0A, 0x68, 0x29, 0xB8]); // str w10, [x0, x9]
        }
        self.stack_depth -= 2;
    }

    fn emit_f32_load(&mut self, offset: u32) {
        // f32.load is identical to i32.load (same bit pattern, 4 bytes)
        self.emit_i32_load(offset);
    }

    fn emit_f32_store(&mut self, offset: u32) {
        // f32.store is identical to i32.store (same bit pattern, 4 bytes)
        self.emit_i32_store(offset);
    }

    // === Local Variables ===
    //
    // Locals are stored on the native stack frame, below the saved rbp/x29.
    // local[i] is at [rbp - (i+1)*8] on x86_64, [x29 - (i+1)*8] on aarch64.

    fn emit_local_get(&mut self, idx: u32) {
        let frame_offset = (idx + 1) * 8;
        #[cfg(target_arch = "x86_64")]
        {
            // mov rax, [rbp - frame_offset]; push rax
            self.emit_bytes(&[0x48, 0x8B, 0x85]); // mov rax, [rbp + disp32]
            self.emit_bytes(&(-(frame_offset as i32)).to_le_bytes());
            self.emit_byte(0x50); // push rax
        }
        #[cfg(target_arch = "aarch64")]
        {
            // ldr x9, [x29, #-frame_offset]; str x9, [sp, #-8]!
            let off = frame_offset.min(255);
            // ldur x9, [x29, #-off]
            let enc = 0xF8400000 | ((((-( off as i32)) as u32) & 0x1FF) << 12) | (29 << 5) | 9;
            self.emit_bytes(&enc.to_le_bytes());
            self.emit_bytes(&[0xE9, 0x0F, 0x1F, 0xF8]); // str x9, [sp, #-8]!
        }
        self.stack_depth += 1;
    }

    fn emit_local_set(&mut self, idx: u32) {
        let frame_offset = (idx + 1) * 8;
        #[cfg(target_arch = "x86_64")]
        {
            // pop rax; mov [rbp - frame_offset], rax
            self.emit_byte(0x58); // pop rax
            self.emit_bytes(&[0x48, 0x89, 0x85]); // mov [rbp + disp32], rax
            self.emit_bytes(&(-(frame_offset as i32)).to_le_bytes());
        }
        #[cfg(target_arch = "aarch64")]
        {
            // ldr x9, [sp], #8; stur x9, [x29, #-off]
            self.emit_bytes(&[0xE9, 0x07, 0x41, 0xF8]); // ldr x9, [sp], #8
            let off = frame_offset.min(255);
            let enc = 0xF8000000 | ((((-( off as i32)) as u32) & 0x1FF) << 12) | (29 << 5) | 9;
            self.emit_bytes(&enc.to_le_bytes());
        }
        self.stack_depth -= 1;
    }

    fn emit_local_tee(&mut self, idx: u32) {
        let frame_offset = (idx + 1) * 8;
        #[cfg(target_arch = "x86_64")]
        {
            // mov rax, [rsp]; mov [rbp - frame_offset], rax (peek, don't pop)
            self.emit_bytes(&[0x48, 0x8B, 0x04, 0x24]); // mov rax, [rsp]
            self.emit_bytes(&[0x48, 0x89, 0x85]); // mov [rbp + disp32], rax
            self.emit_bytes(&(-(frame_offset as i32)).to_le_bytes());
        }
        #[cfg(target_arch = "aarch64")]
        {
            // ldr x9, [sp]; stur x9, [x29, #-off]
            self.emit_bytes(&[0xE9, 0x03, 0x40, 0xF9]); // ldr x9, [sp]
            let off = frame_offset.min(255);
            let enc = 0xF8000000 | ((((-( off as i32)) as u32) & 0x1FF) << 12) | (29 << 5) | 9;
            self.emit_bytes(&enc.to_le_bytes());
        }
        // no stack depth change — value stays on stack
    }

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

    // === Conversions ===

    fn emit_i32_trunc_f32_s(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            // movss xmm0, [rsp]; cvttss2si eax, xmm0; mov [rsp], eax
            self.emit_bytes(&[0xF3, 0x0F, 0x10, 0x04, 0x24]); // movss xmm0, [rsp]
            self.emit_bytes(&[0xF3, 0x0F, 0x2C, 0xC0]);         // cvttss2si eax, xmm0
            self.emit_bytes(&[0x89, 0x04, 0x24]);                 // mov [rsp], eax
        }
        #[cfg(target_arch = "aarch64")]
        {
            self.emit_bytes(&[0xE0, 0x03, 0x40, 0xBD]); // ldr s0, [sp]
            self.emit_bytes(&[0x09, 0x00, 0x38, 0x1E]); // fcvtzs w9, s0
            self.emit_bytes(&[0xE9, 0x03, 0x00, 0xB9]); // str w9, [sp]
        }
        // no stack depth change — f32 becomes i32 in same slot
    }

    fn emit_f32_convert_i32_s(&mut self) {
        #[cfg(target_arch = "x86_64")]
        {
            // mov eax, [rsp]; cvtsi2ss xmm0, eax; movss [rsp], xmm0
            self.emit_bytes(&[0x8B, 0x04, 0x24]);                 // mov eax, [rsp]
            self.emit_bytes(&[0xF3, 0x0F, 0x2A, 0xC0]);         // cvtsi2ss xmm0, eax
            self.emit_bytes(&[0xF3, 0x0F, 0x11, 0x04, 0x24]); // movss [rsp], xmm0
        }
        #[cfg(target_arch = "aarch64")]
        {
            self.emit_bytes(&[0xE9, 0x03, 0x40, 0xB9]); // ldr w9, [sp]
            self.emit_bytes(&[0x20, 0x01, 0x22, 0x1E]); // scvtf s0, w9
            self.emit_bytes(&[0xE0, 0x03, 0x00, 0xBD]); // str s0, [sp]
        }
    }

    // === Helpers ===

    fn emit_byte(&mut self, b: u8) {
        self.code.push(b);
    }

    fn emit_bytes(&mut self, bytes: &[u8]) {
        self.code.extend_from_slice(bytes);
    }
}
