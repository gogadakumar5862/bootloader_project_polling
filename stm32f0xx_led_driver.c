#include "stm32f0xx.h"
#include "stm32f0xx_gpio_driver.h"
#include "stm32f0xx_rcc_driver.h"
#include "stm32f0xx_led_driver.h"

/*********************************************************************
 * @fn      		  - LED_Init
 *
 * @brief             - Initialize PC8 as Output (LED)
 **********************************************************************/
void LED_Init(void)
{
    GPIO_PinConfig_t gpio;

    /* Enable clock for GPIOC */
    RCC_PClkControl_GPIO(GPIOC, ENABLE);

    /* Configure PC8 as OUTPUT */
    gpio.GPIO_PinNumber = LED_PIN;     // PC8 = 8
    gpio.GPIO_PinMode   = 1;           // Output mode
    gpio.GPIO_PinSpeed  = 3;           // High speed
    gpio.GPIO_PinPuPd   = 0;           // No pull-up/down

    GPIO_Init(GPIOC, &gpio);
}

/*********************************************************************
 * @fn      		  - LED_On
 **********************************************************************/
void LED_On(void)
{
    GPIOC->ODR |= (1U << LED_PIN);
}

/*********************************************************************
 * @fn      		  - LED_Off
 **********************************************************************/
void LED_Off(void)
{
    GPIOC->ODR &= ~(1U << LED_PIN);
}

/*********************************************************************
 * @fn      		  - LED_Toggle
 **********************************************************************/
void LED_Toggle(void)
{
    GPIOC->ODR ^= (1U << LED_PIN);
}

/*********************************************************************
 * @fn      		  - LED_Blink
 *
 * @brief             - Blink LED once: ON → delay → OFF
 **********************************************************************/
void LED_Blink(void)
{
    LED_On();

    for (volatile uint32_t i = 0; i < 300000; i++);  // short delay

    LED_Off();
}

