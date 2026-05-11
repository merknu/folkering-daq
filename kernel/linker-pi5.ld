/* Folkering DAQ Kernel Linker Script — Raspberry Pi 5 (bare-metal)
 * Identity-mapped at 0x80000 (Pi firmware default kernel load address).
 * No Limine, no higher-half — Pi firmware loads kernel8.img to this physical
 * address and jumps straight to `_start` (which lives in `.text.boot`).
 *
 * x0 = DTB physical pointer when firmware enters here.
 */

OUTPUT_FORMAT(elf64-littleaarch64)
ENTRY(_start)

SECTIONS {
    /* Pi 5 firmware default kernel-load physical address. */
    . = 0x80000;

    __kernel_start = .;

    /* `.text.boot` MUST be first so `_start` sits at 0x80000. */
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
        /* Limine request markers are unused on Pi-firmware boot but keeping
         * them in this section is harmless and avoids needing duplicate
         * "with-Limine"/"without-Limine" main.rs static layouts. */
        KEEP(*(.requests_start_marker))
        KEEP(*(.requests))
        KEEP(*(.requests_end_marker))
    }

    . = ALIGN(4096);

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
