//! Silverfir-nano — Micro-JIT WASM Runtime for Folkering DAQ
//!
//! A minimal WebAssembly runtime optimized for real-time DSP workloads.
//! Unlike wasmi (interpreter), Silverfir compiles WASM to native machine
//! code at load time for near-native execution speed.
//!
//! Architecture:
//!   1. Parser: Reads WASM binary format (sections, types, functions)
//!   2. Compiler: Translates WASM opcodes → native aarch64/x86_64 code
//!   3. Linker: Resolves host function imports (folk_daq_*, folk_draw_*)
//!   4. Runner: Executes compiled code with access to linear memory
//!
//! Design constraints:
//!   - no_std, no external crate dependencies
//!   - JIT pages use W^X via memory::jit::JitPage
//!   - Linear memory is a contiguous allocation (mappable to ring buffer)
//!   - Hot-swap: new .wasm can replace old without stopping data flow

pub mod parser;
pub mod compiler;
#[cfg(target_arch = "aarch64")]
pub mod aarch64_emit;
pub mod runtime;

use alloc::vec::Vec;
use crate::memory::jit::JitPage;

/// Maximum WASM linear memory: 1 MiB (sufficient for DSP + small buffers)
pub const MAX_LINEAR_MEMORY: usize = 1024 * 1024;

/// Maximum number of compiled functions per module
pub const MAX_FUNCTIONS: usize = 256;

/// Maximum number of host function imports
pub const MAX_IMPORTS: usize = 32;

/// Error type for Silverfir operations
#[derive(Debug)]
pub enum SilverfirError {
    /// WASM binary is malformed
    ParseError(&'static str),
    /// JIT compilation failed
    CompileError(&'static str),
    /// Host function not found during import resolution
    UnresolvedImport,
    /// Out of memory (JIT pages or linear memory)
    OutOfMemory,
    /// Function index out of bounds
    InvalidFunction(u32),
    /// Execution trapped (div by zero, unreachable, etc.)
    Trap(&'static str),
}

/// A compiled WASM module ready for execution
pub struct SilverfirModule {
    /// Compiled native code pages
    pub code_pages: Vec<JitPage>,

    /// Linear memory (WASM "memory" section)
    pub linear_memory: Vec<u8>,

    /// Function table: index → (code_page_idx, offset_in_page)
    pub functions: Vec<CompiledFunction>,

    /// Import table: maps import index → host function pointer
    pub imports: Vec<ImportEntry>,

    /// Exported function names → function index
    pub exports: Vec<ExportEntry>,
}

/// A compiled WASM function
pub struct CompiledFunction {
    /// Index into code_pages
    pub page_index: usize,
    /// Byte offset within the JIT page
    pub code_offset: usize,
    /// Size of compiled code in bytes
    pub code_size: usize,
    /// Number of parameters
    pub param_count: u8,
    /// Whether it returns a value
    pub has_return: bool,
}

/// A resolved import (host function)
pub struct ImportEntry {
    pub module: Vec<u8>,   // "env"
    pub name: Vec<u8>,     // "folk_daq_read_samples"
    pub func_ptr: Option<usize>, // Resolved native function pointer
}

/// An exported function
pub struct ExportEntry {
    pub name: Vec<u8>,
    pub func_index: u32,
}

impl SilverfirModule {
    /// Load and compile a WASM binary.
    ///
    /// This is the main entry point:
    ///   1. Parse the WASM binary to extract types, imports, functions, exports
    ///   2. Resolve imports against registered host functions
    ///   3. JIT-compile each function body to native code
    ///   4. Return a ready-to-execute module
    pub fn load(wasm_bytes: &[u8]) -> Result<Self, SilverfirError> {
        // Phase 1: Parse
        let parsed = parser::parse_module(wasm_bytes)?;

        // Phase 2: Allocate linear memory
        let mem_pages = parsed.memory_pages.unwrap_or(1);
        let mem_size = (mem_pages as usize * 65536).min(MAX_LINEAR_MEMORY);
        let mut linear_memory = Vec::new();
        linear_memory.resize(mem_size, 0u8);

        // Copy data segments into linear memory
        for seg in &parsed.data_segments {
            if seg.offset + seg.data.len() <= linear_memory.len() {
                linear_memory[seg.offset..seg.offset + seg.data.len()]
                    .copy_from_slice(&seg.data);
            }
        }

        // Phase 3: Compile functions
        let mut code_pages = Vec::new();
        let mut functions = Vec::new();

        for (i, func_body) in parsed.function_bodies.iter().enumerate() {
            let type_idx = parsed.function_types.get(i).copied().unwrap_or(0) as usize;
            let func_type = parsed.types.get(type_idx);

            let (param_count, has_return) = match func_type {
                Some(t) => (t.params.len() as u8, !t.results.is_empty()),
                None => (0, false),
            };

            // Compile to native code
            let native_code = compiler::compile_function(func_body, func_type)?;

            // Allocate a JIT page for this function
            let page_size = ((native_code.len() + 4095) / 4096) * 4096;
            let mut page = JitPage::allocate(page_size)
                .ok_or(SilverfirError::OutOfMemory)?;

            page.write_code(0, &native_code);

            // Make the page executable
            unsafe { page.make_executable(); }

            let page_idx = code_pages.len();
            code_pages.push(page);

            functions.push(CompiledFunction {
                page_index: page_idx,
                code_offset: 0,
                code_size: native_code.len(),
                param_count,
                has_return,
            });
        }

        // Phase 4: Build import and export tables
        let imports = parsed.imports.into_iter().map(|imp| {
            ImportEntry {
                module: imp.module,
                name: imp.name,
                func_ptr: None, // Resolved later by register_host_function
            }
        }).collect();

        let exports = parsed.exports.into_iter().map(|exp| {
            ExportEntry {
                name: exp.name,
                func_index: exp.index,
            }
        }).collect();

        Ok(SilverfirModule {
            code_pages,
            linear_memory,
            functions,
            imports,
            exports,
        })
    }

    /// Register a host function to satisfy an import.
    pub fn register_host_function(&mut self, module: &str, name: &str, func_ptr: usize) -> bool {
        for imp in &mut self.imports {
            if imp.module == module.as_bytes() && imp.name == name.as_bytes() {
                imp.func_ptr = Some(func_ptr);
                return true;
            }
        }
        false
    }

    /// Find an exported function by name and return its index.
    pub fn find_export(&self, name: &str) -> Option<u32> {
        for exp in &self.exports {
            if exp.name == name.as_bytes() {
                return Some(exp.func_index);
            }
        }
        None
    }

    /// Call an exported function by index.
    ///
    /// # Safety
    /// - The function must be a valid compiled function
    /// - Arguments must match the function's type signature
    pub unsafe fn call(&mut self, func_index: u32, args: &[i64]) -> Result<Option<i64>, SilverfirError> {
        let idx = func_index as usize;
        let num_imports = self.imports.len();

        if idx < num_imports {
            // This is an imported (host) function
            let imp = &self.imports[idx];
            match imp.func_ptr {
                Some(ptr) => {
                    // Call the host function via function pointer
                    // For now, we support up to 4 arguments
                    let func: fn(&mut [u8], i32, i32, i32, i32) -> i32 =
                        core::mem::transmute(ptr);
                    let a0 = args.get(0).copied().unwrap_or(0) as i32;
                    let a1 = args.get(1).copied().unwrap_or(0) as i32;
                    let a2 = args.get(2).copied().unwrap_or(0) as i32;
                    let a3 = args.get(3).copied().unwrap_or(0) as i32;
                    let result = func(&mut self.linear_memory, a0, a1, a2, a3);
                    Ok(Some(result as i64))
                }
                None => Err(SilverfirError::UnresolvedImport),
            }
        } else {
            // This is a local function
            let local_idx = idx - num_imports;
            let func = self.functions.get(local_idx)
                .ok_or(SilverfirError::InvalidFunction(func_index))?;

            let page = &self.code_pages[func.page_index];
            let code_ptr = page.ptr.add(func.code_offset);

            // Call the JIT-compiled native code
            // The calling convention passes linear_memory ptr and args
            runtime::call_jit_function(
                code_ptr,
                self.linear_memory.as_mut_ptr(),
                self.linear_memory.len(),
                args,
            )
        }
    }
}
