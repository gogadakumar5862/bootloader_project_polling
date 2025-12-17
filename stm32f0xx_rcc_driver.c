#include "stm32f0xx_rcc_driver.h"


/* Minimal implementations. Adjust constants if your board uses different clock tree. */
void RCC_PClkControl_USART(USART_TypeDef *pUSARTx, uint8_t EnOrDi)
{
	if (pUSARTx == USART1)
	{
		if (EnOrDi == ENABLE)
			RCC->APB2ENR |= (1U << 14); // USART1EN
		else
			RCC->APB2ENR &= ~(1U << 14);
	}
	else if (pUSARTx == USART2)
	{
		if (EnOrDi == ENABLE)
			RCC->APB1ENR |= (1U << 17); // USART2EN
		else
			RCC->APB1ENR &= ~(1U << 17);
	}
}


void RCC_PClkControl_GPIO(GPIO_TypeDef *pGPIOx, uint8_t EnOrDi)
{
	if (pGPIOx == GPIOA)
	    {
	        if (EnOrDi == ENABLE)
	            RCC->AHBENR |= (1U << 17);  // GPIOA clock enable
	        else
	            RCC->AHBENR &= ~(1U << 17);
	    }
	    else if (pGPIOx == GPIOC)
	    {
	        if (EnOrDi == ENABLE)
	            RCC->AHBENR |= (1U << 19);  // GPIOC clock enable
	        else
	            RCC->AHBENR &= ~(1U << 19);
	    }
}


/* Very small helper: returns APB1 PCLK. For the Discovery default (HSI/8MHz) this will be approx 8 MHz.
For robust use, adapt to read CFGR and compute actual prescalers. */
uint32_t RCC_GetPCLK1Value(void)
{
	/* Simplified assumption: HSI (8MHz) used, no prescalers */
	return 8000000U;
}


uint32_t RCC_GetHCLKValue(void)
{
	return 8000000U;
}


