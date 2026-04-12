//! WASM Binary Format Parser
//!
//! Parses the WebAssembly binary format (version 1) into an intermediate
//! representation that the compiler can consume.
//!
//! Reference: https://webassembly.github.io/spec/core/binary/
//!
//! We only parse what we need for DAQ workloads:
//!   - Type section (function signatures)
//!   - Import section (host functions)
//!   - Function section (type indices)
//!   - Export section (callable entry points)
//!   - Memory section (linear memory size)
//!   - Code section (function bodies)
//!   - Data section (initial memory contents)

use alloc::vec::Vec;
use super::SilverfirError;

/// WASM magic number: \0asm
const WASM_MAGIC: [u8; 4] = [0x00, 0x61, 0x73, 0x6D];
/// WASM version 1
const WASM_VERSION: [u8; 4] = [0x01, 0x00, 0x00, 0x00];

/// Section IDs
const SEC_TYPE: u8 = 1;
const SEC_IMPORT: u8 = 2;
const SEC_FUNCTION: u8 = 3;
const SEC_MEMORY: u8 = 5;
const SEC_EXPORT: u8 = 7;
const SEC_CODE: u8 = 10;
const SEC_DATA: u8 = 11;

/// WASM value types
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ValType {
    I32 = 0x7F,
    I64 = 0x7E,
    F32 = 0x7D,
    F64 = 0x7C,
}

impl ValType {
    fn from_byte(b: u8) -> Result<Self, SilverfirError> {
        match b {
            0x7F => Ok(ValType::I32),
            0x7E => Ok(ValType::I64),
            0x7D => Ok(ValType::F32),
            0x7C => Ok(ValType::F64),
            _ => Err(SilverfirError::ParseError("unknown value type")),
        }
    }
}

/// Function type (params → results)
#[derive(Debug, Clone)]
pub struct FuncType {
    pub params: Vec<ValType>,
    pub results: Vec<ValType>,
}

/// Parsed import entry
#[derive(Debug, Clone)]
pub struct ParsedImport {
    pub module: Vec<u8>,
    pub name: Vec<u8>,
    pub type_index: u32,
}

/// Parsed export entry
#[derive(Debug, Clone)]
pub struct ParsedExport {
    pub name: Vec<u8>,
    pub kind: u8,   // 0=func, 1=table, 2=memory, 3=global
    pub index: u32,
}

/// Data segment
#[derive(Debug, Clone)]
pub struct DataSegment {
    pub offset: usize,
    pub data: Vec<u8>,
}

/// Complete parsed module
pub struct ParsedModule {
    pub types: Vec<FuncType>,
    pub imports: Vec<ParsedImport>,
    pub function_types: Vec<u32>,  // function index → type index
    pub exports: Vec<ParsedExport>,
    pub memory_pages: Option<u32>,
    pub function_bodies: Vec<Vec<u8>>, // raw bytecode per function
    pub data_segments: Vec<DataSegment>,
}

/// Cursor for reading the binary
struct Reader<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn new(data: &'a [u8]) -> Self {
        Reader { data, pos: 0 }
    }

    fn remaining(&self) -> usize {
        self.data.len() - self.pos
    }

    fn read_byte(&mut self) -> Result<u8, SilverfirError> {
        if self.pos >= self.data.len() {
            return Err(SilverfirError::ParseError("unexpected EOF"));
        }
        let b = self.data[self.pos];
        self.pos += 1;
        Ok(b)
    }

    fn read_bytes(&mut self, n: usize) -> Result<&'a [u8], SilverfirError> {
        if self.pos + n > self.data.len() {
            return Err(SilverfirError::ParseError("unexpected EOF in bytes"));
        }
        let slice = &self.data[self.pos..self.pos + n];
        self.pos += n;
        Ok(slice)
    }

    fn read_u32_le(&mut self) -> Result<u32, SilverfirError> {
        let bytes = self.read_bytes(4)?;
        Ok(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
    }

    /// Read LEB128-encoded unsigned integer
    fn read_u32_leb128(&mut self) -> Result<u32, SilverfirError> {
        let mut result: u32 = 0;
        let mut shift = 0;
        loop {
            let byte = self.read_byte()?;
            result |= ((byte & 0x7F) as u32) << shift;
            if byte & 0x80 == 0 { break; }
            shift += 7;
            if shift >= 35 {
                return Err(SilverfirError::ParseError("LEB128 overflow"));
            }
        }
        Ok(result)
    }

    /// Read LEB128-encoded signed integer (for i32 constants)
    fn read_i32_leb128(&mut self) -> Result<i32, SilverfirError> {
        let mut result: i32 = 0;
        let mut shift = 0;
        let mut byte;
        loop {
            byte = self.read_byte()?;
            result |= ((byte & 0x7F) as i32) << shift;
            shift += 7;
            if byte & 0x80 == 0 { break; }
            if shift >= 35 {
                return Err(SilverfirError::ParseError("signed LEB128 overflow"));
            }
        }
        // Sign extend
        if shift < 32 && (byte & 0x40) != 0 {
            result |= !0 << shift;
        }
        Ok(result)
    }

    /// Read a name (length-prefixed UTF-8 string as bytes)
    fn read_name(&mut self) -> Result<Vec<u8>, SilverfirError> {
        let len = self.read_u32_leb128()? as usize;
        let bytes = self.read_bytes(len)?;
        Ok(bytes.to_vec())
    }

    fn skip(&mut self, n: usize) -> Result<(), SilverfirError> {
        if self.pos + n > self.data.len() {
            return Err(SilverfirError::ParseError("skip past EOF"));
        }
        self.pos += n;
        Ok(())
    }
}

/// Parse a complete WASM module from binary bytes
pub fn parse_module(bytes: &[u8]) -> Result<ParsedModule, SilverfirError> {
    let mut r = Reader::new(bytes);

    // Validate magic and version
    let magic = r.read_bytes(4)?;
    if magic != WASM_MAGIC {
        return Err(SilverfirError::ParseError("invalid WASM magic"));
    }
    let version = r.read_bytes(4)?;
    if version != WASM_VERSION {
        return Err(SilverfirError::ParseError("unsupported WASM version"));
    }

    let mut module = ParsedModule {
        types: Vec::new(),
        imports: Vec::new(),
        function_types: Vec::new(),
        exports: Vec::new(),
        memory_pages: None,
        function_bodies: Vec::new(),
        data_segments: Vec::new(),
    };

    // Parse sections
    while r.remaining() > 0 {
        let section_id = r.read_byte()?;
        let section_size = r.read_u32_leb128()? as usize;
        let section_end = r.pos + section_size;

        match section_id {
            SEC_TYPE => parse_type_section(&mut r, &mut module)?,
            SEC_IMPORT => parse_import_section(&mut r, &mut module)?,
            SEC_FUNCTION => parse_function_section(&mut r, &mut module)?,
            SEC_MEMORY => parse_memory_section(&mut r, &mut module)?,
            SEC_EXPORT => parse_export_section(&mut r, &mut module)?,
            SEC_CODE => parse_code_section(&mut r, &mut module)?,
            SEC_DATA => parse_data_section(&mut r, &mut module)?,
            _ => {
                // Skip unknown sections
                r.skip(section_size)?;
            }
        }

        // Ensure we consumed exactly the section
        if r.pos != section_end {
            r.pos = section_end; // Recover by skipping to end
        }
    }

    Ok(module)
}

fn parse_type_section(r: &mut Reader, module: &mut ParsedModule) -> Result<(), SilverfirError> {
    let count = r.read_u32_leb128()?;
    for _ in 0..count {
        let form = r.read_byte()?;
        if form != 0x60 {
            return Err(SilverfirError::ParseError("expected functype marker 0x60"));
        }

        // Params
        let param_count = r.read_u32_leb128()?;
        let mut params = Vec::with_capacity(param_count as usize);
        for _ in 0..param_count {
            params.push(ValType::from_byte(r.read_byte()?)?);
        }

        // Results
        let result_count = r.read_u32_leb128()?;
        let mut results = Vec::with_capacity(result_count as usize);
        for _ in 0..result_count {
            results.push(ValType::from_byte(r.read_byte()?)?);
        }

        module.types.push(FuncType { params, results });
    }
    Ok(())
}

fn parse_import_section(r: &mut Reader, module: &mut ParsedModule) -> Result<(), SilverfirError> {
    let count = r.read_u32_leb128()?;
    for _ in 0..count {
        let module_name = r.read_name()?;
        let field_name = r.read_name()?;
        let kind = r.read_byte()?;

        match kind {
            0x00 => {
                // Function import
                let type_index = r.read_u32_leb128()?;
                module.imports.push(ParsedImport {
                    module: module_name,
                    name: field_name,
                    type_index,
                });
            }
            0x01 => {
                // Table import — skip
                r.read_byte()?; // elemtype
                r.read_u32_leb128()?; // limits flag
                r.read_u32_leb128()?; // min
                // If flag has max: read another u32
            }
            0x02 => {
                // Memory import — skip
                let flags = r.read_u32_leb128()?;
                r.read_u32_leb128()?; // min
                if flags & 1 != 0 { r.read_u32_leb128()?; } // max
            }
            0x03 => {
                // Global import — skip
                r.read_byte()?; // valtype
                r.read_byte()?; // mutability
            }
            _ => return Err(SilverfirError::ParseError("unknown import kind")),
        }
    }
    Ok(())
}

fn parse_function_section(r: &mut Reader, module: &mut ParsedModule) -> Result<(), SilverfirError> {
    let count = r.read_u32_leb128()?;
    for _ in 0..count {
        let type_index = r.read_u32_leb128()?;
        module.function_types.push(type_index);
    }
    Ok(())
}

fn parse_memory_section(r: &mut Reader, module: &mut ParsedModule) -> Result<(), SilverfirError> {
    let count = r.read_u32_leb128()?;
    if count > 0 {
        let flags = r.read_u32_leb128()?;
        let min_pages = r.read_u32_leb128()?;
        if flags & 1 != 0 {
            let _max_pages = r.read_u32_leb128()?;
        }
        module.memory_pages = Some(min_pages);
    }
    Ok(())
}

fn parse_export_section(r: &mut Reader, module: &mut ParsedModule) -> Result<(), SilverfirError> {
    let count = r.read_u32_leb128()?;
    for _ in 0..count {
        let name = r.read_name()?;
        let kind = r.read_byte()?;
        let index = r.read_u32_leb128()?;
        module.exports.push(ParsedExport { name, kind, index });
    }
    Ok(())
}

fn parse_code_section(r: &mut Reader, module: &mut ParsedModule) -> Result<(), SilverfirError> {
    let count = r.read_u32_leb128()?;
    for _ in 0..count {
        let body_size = r.read_u32_leb128()? as usize;
        let body_bytes = r.read_bytes(body_size)?;
        module.function_bodies.push(body_bytes.to_vec());
    }
    Ok(())
}

fn parse_data_section(r: &mut Reader, module: &mut ParsedModule) -> Result<(), SilverfirError> {
    let count = r.read_u32_leb128()?;
    for _ in 0..count {
        let flags = r.read_u32_leb128()?;
        if flags == 0 {
            // Active segment with memory index 0
            // Read the offset expression (typically i32.const + end)
            let opcode = r.read_byte()?;
            let offset = if opcode == 0x41 {
                // i32.const
                r.read_i32_leb128()? as usize
            } else {
                0
            };
            let end_opcode = r.read_byte()?;
            if end_opcode != 0x0B {
                return Err(SilverfirError::ParseError("expected end opcode in data init expr"));
            }

            let data_len = r.read_u32_leb128()? as usize;
            let data = r.read_bytes(data_len)?.to_vec();

            module.data_segments.push(DataSegment { offset, data });
        } else {
            // Passive or other segment types — skip
            let data_len = r.read_u32_leb128()? as usize;
            r.skip(data_len)?;
        }
    }
    Ok(())
}
