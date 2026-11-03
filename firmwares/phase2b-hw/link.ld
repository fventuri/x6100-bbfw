/* Linker script for the X6100 BASE (STM32F427) application image.
 *
 * Vector table / entry at 0x08020000: the 128 KiB IAP bootloader occupies
 *   [0x08000000, 0x08020000) below us (Oleg startup.md: "Vector-table address
 *   0x08020000, Initial MSP 0x20030000" VERIFIED).
 * SRAM1+SRAM2+SRAM3 = 192 KiB contiguous at 0x20000000; the OEM initial MSP
 *   (0x20030000) is the top of this region and the stack grows down from it.
 */

ENTRY(Reset_Handler)

_estack = 0x20030000;          /* top of 192 KiB contiguous SRAM (OEM MSP) */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08020000, LENGTH = 384K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 192K
}

SECTIONS
{
    .isr_vector : {
        KEEP(*(.isr_vector))
    } > FLASH

    .text : {
        *(.text*)
        *(.rodata*)
    } > FLASH

    .data : {
        . = ALIGN(4);
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH
    _sidata = LOADADDR(.data);

    .bss : {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    /* Heap base for newlib-nano (snprintf family pulls in _sbrk). */
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    PROVIDE(__end__ = .);

    /DISCARD/ : {
        *(.ARM.exidx*)
        *(.ARM.extab*)
        *(.comment)
    }
}
