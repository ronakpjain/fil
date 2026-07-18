ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 4K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 4K
}

PHDRS
{
    flash PT_LOAD FLAGS(5);
    data  PT_LOAD FLAGS(6);
    bss   PT_LOAD FLAGS(6);
}

SECTIONS
{
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } > FLASH :flash

    .text :
    {
        *(.text*)
        *(.rodata*)
    } > FLASH :flash

    .data :
    {
        *(.data*)
    } > RAM AT> FLASH :data

    .bss (NOLOAD) :
    {
        _sbss = .;
        *(.bss*)
        _ebss = .;
    } > RAM :bss

    _estack = ORIGIN(RAM) + LENGTH(RAM);

    .ARM.attributes 0 :
    {
        *(.ARM.attributes)
    }
}
