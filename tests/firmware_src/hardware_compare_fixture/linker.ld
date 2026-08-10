ENTRY(Reset_Handler)

/* Minimal STM32G4 flash/RAM image used only for hardware comparison. */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 4K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 4K
}

PHDRS
{
    flash PT_LOAD FLAGS(5);
    data  PT_LOAD FLAGS(6);
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

    .ARM.attributes 0 :
    {
        *(.ARM.attributes)
    }
}
