ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 4K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 256
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
        _sdata = .;
        *(.data*)
        _edata = .;
    } > RAM AT> FLASH :data
    _sidata = LOADADDR(.data);

    .bss (NOLOAD) :
    {
        _sbss = .;
        *(.bss*)
        _ebss = .;
    } > RAM :bss

    .ARM.attributes 0 :
    {
        *(.ARM.attributes)
    }
}
