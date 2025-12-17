#ifndef STM32F0XX_RCC_DRIVER_H
#define STM32F0XX_RCC_DRIVER_H


#include <stdint.h>
#include "stm32f0xx.h"


/* Peripheral clock control */
void RCC_PClkControl_USART(USART_TypeDef *pUSARTx, uint8_t EnOrDi);
void RCC_PClkControl_GPIO(GPIO_TypeDef *pGPIOx, uint8_t EnOrDi);
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetHCLKValue(void);


#endif // STM32F0XX_RCC_DRIVER_H
