ENTRY(Reset_Handler)

MEMORY
{
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 6k
}

SECTIONS
{
    /* Vector table at the beginning of RAM */
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } > RAM

    /* Program code in RAM */
    .text :
    {
        *(.text*)
        *(.rodata*)
        _etext = .;
    } > RAM

    /* Initialized data already in RAM */
    .data :
    {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > RAM

    /* Uninitialized data */
    .bss :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > RAM

    /* Stack at top of RAM */
    .stack (COPY):
    {
        . = ALIGN(8);
        _estack = ORIGIN(RAM) + LENGTH(RAM);
    } > RAM
}

