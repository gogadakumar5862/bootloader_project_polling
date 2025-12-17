#include <stdint.h>

/* Symbols defined in linker script */
extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _estack;

/* Function prototypes */
int main(void);
void Reset_Handler(void);
void Default_Handler(void);

/* Cortex-M4 core exception handlers */
void NMI_Handler(void)            __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)      __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)       __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)            __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)       __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)         __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)        __attribute__((weak, alias("Default_Handler")));

/* Vector table */
__attribute__((section(".isr_vector")))
uint32_t *vector_table[] =
{
    (uint32_t*)(&_estack),   /* Initial stack pointer */
    (uint32_t*)Reset_Handler, /* Reset handler */
    (uint32_t*)NMI_Handler,
    (uint32_t*)HardFault_Handler,
    (uint32_t*)MemManage_Handler,
    (uint32_t*)BusFault_Handler,
    (uint32_t*)UsageFault_Handler,
    0, 0, 0, 0,              /* Reserved */
    (uint32_t*)SVC_Handler,
    (uint32_t*)DebugMon_Handler,
    0,                       /* Reserved */
    (uint32_t*)PendSV_Handler,
    (uint32_t*)SysTick_Handler
    /* You can add IRQ handlers for peripherals here */
};

/* Reset handler */
void Reset_Handler(void)
{
    uint32_t *src, *dst;

    /* Copy .data from FLASH to RAM */
/*    src = &_etext;
    dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }
*/
    /* Zero initialize .bss */
    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
    }

    /* Call main */
    main();

    /* If main() returns, loop forever */
    while (1);
}

/* Default handler for unused IRQs */
void Default_Handler(void)
{
    while (1);
}

