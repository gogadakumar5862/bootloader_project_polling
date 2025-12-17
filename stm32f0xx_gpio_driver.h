#ifndef STM32F0XX_GPIO_DRIVER_H
#define STM32F0XX_GPIO_DRIVER_H


#include "stm32f0xx.h"
#include <stdint.h>


typedef struct {
uint8_t GPIO_PinNumber; /* 0..15 */
uint8_t GPIO_PinMode; /* 0=input,1=output,2=alt,3=analog */
uint8_t GPIO_PinSpeed; /* 0..3 */
uint8_t GPIO_PinPuPd; /* 0=no,1=pulldown,2=pullup */
uint8_t GPIO_PinAltFun; /* AF number if AF mode */
} GPIO_PinConfig_t;


void GPIO_Init(GPIO_TypeDef *pGPIOx, GPIO_PinConfig_t *pPinConfig);
void GPIO_DeInit(GPIO_TypeDef *pGPIOx);

/* Helper to set AF for pin */
void GPIO_SetAltFunction(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, uint8_t AltFun);


#endif // STM32F0XX_GPIO_DRIVER_H

