#include "stm32f0xx.h"
#include "stm32f0xx_gpio_driver.h"
#include "stm32f0xx_rcc_driver.h"
#include "stm32f0xx_led_driver.h"

/* -------- externs from bootloader.c -------- */
extern unsigned long ymodem_receive_to_ram(uint8_t *dst_addr,
                                           uint32_t max_size,
                                           uint32_t *received_size);

extern void USART_Init(void *pUSARTHandle);

/* USART handle defined in bootloader.c */
extern struct {
    USART_TypeDef *pUSARTx;
    struct {
        uint8_t USART_Mode;
        uint32_t USART_Baud;
        uint8_t USART_NoOfStopBits;
        uint8_t USART_WordLength;
        uint8_t USART_ParityControl;
    } USART_Config;
} usart1Handle;

/* -------- bootloader configuration -------- */
#define RAM_APP_ADDR        0x20000000
#define MAX_FIRMWARE_SIZE  (4 * 1024U)

/* ============================================================
   USART1 GPIO Initialization
   ============================================================ */
void USART1_GPIOInit(void)
{
    GPIO_PinConfig_t gpio;

    RCC_PClkControl_GPIO(GPIOA, ENABLE);

    /* PA9  -> USART1_TX */
    gpio.GPIO_PinNumber = 9;
    gpio.GPIO_PinMode   = 2;   // AF
    gpio.GPIO_PinSpeed  = 3;
    gpio.GPIO_PinPuPd   = 0;
    GPIO_Init(GPIOA, &gpio);
    GPIO_SetAltFunction(GPIOA, 9, 1);

    /* PA10 -> USART1_RX */
    gpio.GPIO_PinNumber = 10;
    gpio.GPIO_PinMode   = 2;   // AF
    gpio.GPIO_PinSpeed  = 3;
    gpio.GPIO_PinPuPd   = 0;
    GPIO_Init(GPIOA, &gpio);
    GPIO_SetAltFunction(GPIOA, 10, 1);
}

/* ============================================================
   Jump to application in RAM
   ============================================================ */
void jump_to_application(uint32_t app_addr)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->CFGR1 = (3 << SYSCFG_CFGR1_MEM_MODE_Pos);

    uint32_t new_msp = *(uint32_t*)app_addr;
    uint32_t new_pc  = *(uint32_t*)(app_addr + 4);

    __disable_irq();
    __set_MSP(new_msp);
    ((void (*)(void))new_pc)();
}

/* ============================================================
   MAIN
   ============================================================ */
int main(void)
{
    USART1_GPIOInit();

    usart1Handle.pUSARTx = USART1;
    usart1Handle.USART_Config.USART_Baud          = 115200;
    usart1Handle.USART_Config.USART_Mode          = 3;
    usart1Handle.USART_Config.USART_WordLength    = 8;
    usart1Handle.USART_Config.USART_ParityControl = 0;
    usart1Handle.USART_Config.USART_NoOfStopBits  = 1;

    USART_Init(&usart1Handle);

    LED_Init();

    uint32_t received_size = 0;
    uint8_t *ram = (uint8_t *)RAM_APP_ADDR;

    unsigned long ret =
        ymodem_receive_to_ram(ram, MAX_FIRMWARE_SIZE, &received_size);

    if (ret != 0)
    {
        while (1);   // error halt
    }

    jump_to_application(RAM_APP_ADDR);

    while (1);
}

