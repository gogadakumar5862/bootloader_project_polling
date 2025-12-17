ENTRY(reset_handler)
MEMORY
{
    flash (rx) : ORIGIN = 0x08000000, LENGTH = 8K
    /*ram   (rwx): ORIGIN = 0x20000000, LENGTH = 8K*/
    ram  (rwx) : ORIGIN = 0x20001800, LENGTH = 2K

}

SECTIONS
{
    /* --- Vector Table + Code + Const Data --- */
    .text :
    {
        _stext = .;                /* Start of text section */
        . = ALIGN(4);
        KEEP(*(.isr_vector))       /* Keep vector table even if unused */
        . = ALIGN(4);
        *(.text*)                  /* Program code */
        *(.rodata*)                /* Read-only data (consts, strings) */
        . = ALIGN(4);
        _etext = .;                /* End of text section (used as start of .data in flash) */
    } > flash

    /* --- Initialized Data (.data) --- */
    .data : AT(_etext)
    {
        . = ALIGN(4);
        _sdata = .;                /* Start of .data in RAM */
        *(.data*)                  /* Initialized global/static vars */
        . = ALIGN(4);
        _edata = .;                /* End of .data in RAM */
    } > ram
    _sidata = LOADADDR(.data);     /* Flash address for copy source */

    /* --- Zero-initialized Data (.bss) --- */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;                 /* Start of .bss in RAM */
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;                 /* End of .bss in RAM */
    } > ram

    /* --- Optional: Stack symbol (not required but useful) --- */
    _estack = ORIGIN(ram) + LENGTH(ram);
}

