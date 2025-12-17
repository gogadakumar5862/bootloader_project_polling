#include "stm32f0xx.h"
#include <stdint.h>

/*#define SRAM_START  0x20000000U
#define SRAM_SIZE   (8U * 1024U)
#define SRAM_END    (SRAM_START + SRAM_SIZE)
#define STACK_START SRAM_END
*/
#define SRAM_START  0x20001800U
#define SRAM_SIZE   (2U * 1024U)
#define SRAM_END    (SRAM_START + SRAM_SIZE)
#define STACK_START SRAM_END

extern int main(void);
extern uint32_t _estack, _etext, _sdata, _edata, _sidata, _sbss, _ebss;


void reset_handler(void);
void default_handler(void);

void nmi_handler(void)              __attribute__((weak, alias("default_handler")));
void hard_fault_handler(void)       __attribute__((weak, alias("default_handler")));
void mem_manage_handler(void)       __attribute__((weak, alias("default_handler")));
void bus_fault_handler(void)        __attribute__((weak, alias("default_handler")));
void usage_fault_handler(void)      __attribute__((weak, alias("default_handler")));
void svcall_handler(void)           __attribute__((weak, alias("default_handler")));
void debug_monitor_handler(void)    __attribute__((weak, alias("default_handler")));
void pendsv_handler(void)           __attribute__((weak, alias("default_handler")));
void systick_handler(void)          __attribute__((weak, alias("default_handler")));

/* ---- ADD THIS FOR UART INTERRUPT ---- */
void USART1_IRQHandler(void)         __attribute__((weak, alias("default_handler")));

/* Minimal set for blinky */
void exti0_handler(void)            __attribute__((weak, alias("default_handler")));

/* Vector Table */
__attribute__((section(".isr_vector")))
uint32_t vector_table[] = {
    STACK_START,
    (uint32_t)reset_handler,
    (uint32_t)nmi_handler,
    (uint32_t)hard_fault_handler,
    (uint32_t)mem_manage_handler,
    (uint32_t)bus_fault_handler,
    (uint32_t)usage_fault_handler,
    0, 0, 0, 0,
    (uint32_t)svcall_handler,
    (uint32_t)debug_monitor_handler,
    0,
    (uint32_t)pendsv_handler,
    (uint32_t)systick_handler,
    /* IRQ handlers start */
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    (uint32_t)USART1_IRQHandler,
    //(uint32_t)exti0_handler,  /* placeholder for first IRQ */
    /* Remaining IRQs defaulted in real vector table */
};

void reset_handler(void)
{
    /* Copy .data from FLASH to SRAM */
    uint32_t size = (uint32_t)&_edata - (uint32_t)&_sdata;
    uint8_t *dst = (uint8_t*)&_sdata;
    uint8_t *src = (uint8_t*)&_sidata;
    for (uint32_t i = 0; i < size; i++)
        dst[i] = src[i];

    /* Zero fill .bss */
    size = (uint32_t)&_ebss - (uint32_t)&_sbss;
    dst = (uint8_t*)&_sbss;
    for (uint32_t i = 0; i < size; i++)
        dst[i] = 0;

    /* Call main */
    main();
}

void default_handler(void)
{
    while (1);
}
