/* Folkering DAQ Kernel Linker Script — QEMU virt
 * Flat binary, identity-mapped at 0x4008_0000 (QEMU virt load address)
 * No higher-half — QEMU loads kernel directly without bootloader
 */

OUTPUT_FORMAT(elf64-littleaarch64)
ENTRY(kmain)

SECTIONS {
    /* QEMU virt loads kernel at 0x4008_0000 */
    . = 0x40080000;

    __kernel_start = .;

    .text : {
        KEEP(*(.text.boot))
        *(.text .text.*)
    }

    . = ALIGN(4096);

    .rodata : {
        *(.rodata .rodata.*)
        *(.eh_frame_hdr)
        *(.eh_frame)
    }

    . = ALIGN(4096);

    .data : {
        *(.data .data.*)
        KEEP(*(.requests_start_marker))
        KEEP(*(.requests))
        KEEP(*(.requests_end_marker))
    }

    .bss : {
        PROVIDE(__bss_start = .);
        *(.bss .bss.*)
        *(COMMON)
        PROVIDE(__bss_end = .);
    }

    __kernel_end = .;

    /DISCARD/ : {
        *(.note.GNU-stack)
        *(.note .note.*)
        *(.comment)
    }
}
