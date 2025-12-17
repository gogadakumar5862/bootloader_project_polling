#ifndef STM32F0XX_USART_DRIVER_H
#define STM32F0XX_USART_DRIVER_H


#include "stm32f0xx.h"
#include <stdint.h>


#define USART_READY        0
#define USART_BUSY_IN_TX   1
#define USART_BUSY_IN_RX   2


/* Configuration structure */
typedef struct {
uint32_t USART_Baud;
uint8_t USART_Mode; /* TX/RX */
uint8_t USART_WordLength; /* 8 or 9 */
uint8_t USART_ParityControl; /* 0 none, 1 even, 2 odd */
uint8_t USART_NoOfStopBits; /* 1 or 2 */
uint8_t USART_HWFlowControl; /* disabled for polling-only */
} USART_Config_t;


/* Handle structure */
typedef struct {
USART_TypeDef *pUSARTx;
USART_Config_t USART_Config;

/* Interrupt fields */
uint8_t *pTxBuffer;
uint8_t *pRxBuffer;
uint32_t TxLen;
uint32_t RxLen;
uint8_t TxBusyState;
uint8_t RxBusyState;
} USART_Handle_t;


/* Clock control */
void USART_PeriClockControl(USART_TypeDef *pUSARTx, uint8_t EnOrDi);


/* Init/DeInit */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_TypeDef *pUSARTx);


/* Data send/receive (polling) */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);



/* Interrupt mode */
void USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);


void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);



/* Flags */
uint8_t USART_GetFlagStatus(USART_TypeDef *pUSARTx, uint32_t FlagName);


/* Flags names */
#define USART_FLAG_TXE (1U << 7)
#define USART_FLAG_TC (1U << 6)
#define USART_FLAG_RXNE (1U << 5)


#endif // STM32F0XX_USART_DRIVER_H
