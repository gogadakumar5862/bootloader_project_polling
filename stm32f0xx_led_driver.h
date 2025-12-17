/*********************************************************************
 *    stm32f0xx_led_driver.h
 *    LED driver header for PC8 on STM32F051 Discovery
 *********************************************************************/

#ifndef STM32F0XX_LED_DRIVER_H
#define STM32F0XX_LED_DRIVER_H

#include "stm32f0xx.h"

#define LED_PORT   GPIOC
#define LED_PIN    8        // PC8 = LED4 (Blue LED)

void LED_Init(void);
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);
void LED_Blink(void);

#endif
