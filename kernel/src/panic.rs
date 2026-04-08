//! Panic handler for Folkering DAQ kernel

use core::panic::PanicInfo;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    crate::kprintln!("\n!!! KERNEL PANIC !!!");
    crate::kprintln!("{}", info);

    loop {
        crate::arch::aarch64::wfe();
    }
}

#[alloc_error_handler]
fn alloc_error(layout: core::alloc::Layout) -> ! {
    crate::kprintln!("ALLOC ERROR: {:?}", layout);
    loop {
        crate::arch::aarch64::wfe();
    }
}
