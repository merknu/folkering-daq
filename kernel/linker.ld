/* Folkering DAQ Kernel Linker Script (aarch64)
 * Higher-half kernel at 0xFFFFFFFF80000000
 * Limine boot protocol compatible
 */

OUTPUT_FORMAT(elf64-littleaarch64)

ENTRY(kmain)

PHDRS {
    text PT_LOAD;
    rodata PT_LOAD;
    data PT_LOAD;
}

SECTIONS {
    /* Higher-half kernel base — required by Limine spec */
    . = 0xFFFFFFFF80000000;

    __kernel_start = .;

    /* Text section — executable code */
    .text : {
        KEEP(*(.text.boot))
        *(.text .text.*)
    } :text

    . = ALIGN(CONSTANT(MAXPAGESIZE));

    /* Read-only data */
    .rodata : {
        *(.rodata .rodata.*)
        *(.eh_frame_hdr)
        *(.eh_frame)
    } :rodata

    . = ALIGN(CONSTANT(MAXPAGESIZE));

    /* Read-write data — Limine requests embedded here */
    .data : {
        *(.data .data.*)

        KEEP(*(.requests_start_marker))
        KEEP(*(.requests))
        KEEP(*(.requests_end_marker))
    } :data

    /* BSS — must be last in :data segment */
    .bss : {
        PROVIDE(__bss_start = .);
        *(.bss .bss.*)
        *(COMMON)
        PROVIDE(__bss_end = .);
    } :data

    __kernel_end = .;

    /DISCARD/ : {
        *(.note.GNU-stack)
        *(.note .note.*)
        *(.comment)
    }
}
