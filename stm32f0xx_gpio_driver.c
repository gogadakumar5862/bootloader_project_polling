#include "stm32f0xx_gpio_driver.h"
#include "stm32f0xx_rcc_driver.h"


void GPIO_Init(GPIO_TypeDef *pGPIOx, GPIO_PinConfig_t *pPinConfig)
{
	/* Enable GPIO clock */
	if (pGPIOx == GPIOA) RCC_PClkControl_GPIO(GPIOA, ENABLE);


	uint32_t pos = pPinConfig->GPIO_PinNumber * 2U;


	/* MODER */
	pGPIOx->MODER &= ~(0x3U << pos);
	pGPIOx->MODER |= ((pPinConfig->GPIO_PinMode & 0x3U) << pos);


	/* OTYPER: 0 push-pull (default) */
	/* OSPEEDR */
	pGPIOx->OSPEEDR &= ~(0x3U << pos);
	pGPIOx->OSPEEDR |= ((pPinConfig->GPIO_PinSpeed & 0x3U) << pos);


	/* PUPDR */
	pGPIOx->PUPDR &= ~(0x3U << pos);
	pGPIOx->PUPDR |= ((pPinConfig->GPIO_PinPuPd & 0x3U) << pos);


/* Alternate function: STM32F0 uses AFR[0]/AFR[1] as AFR[0..1] -> AFR[Pin/8]
but in stm32f051x8.h the register is AFR[2] named AFR[] (GPIO_TypeDef->AFR[2])
Use helper to set AFRO for pin when needed. */
}


void GPIO_DeInit(GPIO_TypeDef *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		RCC->AHBRSTR |= (1U << 17);
		RCC->AHBRSTR &= ~(1U << 17);
	}
}


void GPIO_SetAltFunction(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, uint8_t AltFun)
{
	uint8_t idx = PinNumber >> 3; /* 0 for pins 0..7, 1 for 8..15 */
	uint8_t shift = (PinNumber & 0x7U) * 4U;


	pGPIOx->AFR[idx] &= ~(0xFU << shift);
	pGPIOx->AFR[idx] |= ((AltFun & 0xFU) << shift);
}
