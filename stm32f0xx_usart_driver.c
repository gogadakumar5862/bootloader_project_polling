#include "stm32f0xx_usart_driver.h"
#include "stm32f0xx_rcc_driver.h"
#include "stm32f0xx_gpio_driver.h"


/* Peripheral clock control  using RCC driver */
void USART_PeriClockControl(USART_TypeDef *pUSARTx, uint8_t EnOrDi)
{
	RCC_PClkControl_USART(pUSARTx, EnOrDi);
}


/* Simple BRR computation for oversampling by 16 */
static uint32_t USART_ComputeBRR(uint32_t pclk, uint32_t baud)
{
	if (baud == 0) return 0;
	return (pclk + (baud/2U)) / baud;
}

/***************************************************************************
 *                    - USART_Init
 *                    Initializes the USART peripheral
 **************************************************************************/

void USART_Init(USART_Handle_t *pUSARTHandle)
{
	USART_TypeDef *pUSARTx = pUSARTHandle->pUSARTx;
	USART_Config_t *cfg = &pUSARTHandle->USART_Config;


	/* 1. Enable peripheral clock */
	USART_PeriClockControl(pUSARTx, ENABLE);


	/* 2. Configure GPIO pins externally (caller), but we provide a typical mapping example in comments.
	Example for USART1 on STM32F0 Discovery: PA9 = TX (AF0/AF1 depending), PA10 = RX */


	/* 3. Disable USART before configuration */
	pUSARTx->CR1 &= ~USART_CR1_UE;


	/* 4. Word length */
	if (cfg->USART_WordLength == 9)
		pUSARTx->CR1 |= (1U << 12); /* M bit typical position in F0 header */
	else
		pUSARTx->CR1 &= ~(1U << 12);


	/* 5. Parity */
	if (cfg->USART_ParityControl == 0)
	{
		pUSARTx->CR1 &= ~USART_CR1_PCE;
	}
	else
	{
		pUSARTx->CR1 |= USART_CR1_PCE;
		/* PS bit for odd parity (bit 9) */
		if (cfg->USART_ParityControl == 2)
		{
			pUSARTx->CR1 |= (1U << 9); /* PS */
		}
		else
		{
			pUSARTx->CR1 &= ~(1U << 9);
		}
	}


	/* Stop bits: CR2 STOP[13:12] - for simplicity we handle only 1 or 2 */
	if (cfg->USART_NoOfStopBits == 2)
	{
		pUSARTx->CR2 |= (0x2U << 12);
	}
	else
	{
		pUSARTx->CR2 &= ~(0x3U << 12);
	}


	/* Baud rate: compute using PCLK1 or PCLK2 depending on USART instance */
	uint32_t pclk = RCC_GetPCLK1Value();
	if (pUSARTx == USART1) pclk = RCC_GetHCLKValue(); // USART1 on APB2 in F0
	pUSARTx->BRR = USART_ComputeBRR(pclk, cfg->USART_Baud);


	/* Enable transmitter and receiver */
	pUSARTx->CR1 |= (USART_CR1_TE | USART_CR1_RE);


	/* Enable USART */
	pUSARTx->CR1 |= USART_CR1_UE;
}

void USART_DeInit(USART_TypeDef *pUSARTx)
{
	/* Reset registers to default by toggling reset bits in RCC */
	if (pUSARTx == USART1)
	{
		RCC->APB2RSTR |= (1U << 14);
		RCC->APB2RSTR &= ~(1U << 14);
	}
	else if (pUSARTx == USART2)
	{
		RCC->APB1RSTR |= (1U << 17);
		RCC->APB1RSTR &= ~(1U << 17);
	}
}


uint8_t USART_GetFlagStatus(USART_TypeDef *pUSARTx, uint32_t FlagName)
{
	return ((pUSARTx->ISR & FlagName) ? 1U : 0U);
}


void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	USART_TypeDef *pUSARTx = pUSARTHandle->pUSARTx;


	for (uint32_t i = 0; i < Len; ++i)
	{
		/* Wait for TXE */
		while (!USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE)) { }


		/* Write to TDR (16-bit register) */
		pUSARTx->TDR = (uint16_t)(pTxBuffer[i] & 0xFFU);
	}


	/* Optionally wait for TC if caller needs full frame transmitted */
	while (!USART_GetFlagStatus(pUSARTx, USART_FLAG_TC)) { }
}


void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	USART_TypeDef *pUSARTx = pUSARTHandle->pUSARTx;


	for (uint32_t i = 0; i < Len; ++i)
	{
		/* Wait until RXNE */
		while (!USART_GetFlagStatus(pUSARTx, USART_FLAG_RXNE)) { }


		/* Read from RDR (16-bit) */
		pRxBuffer[i] = (uint8_t)(pUSARTx->RDR & 0xFFU);
	}
}

/*********************************************************************
 * 		     		  - USART_SendDataIT (Interrupt TX)
 **********************************************************************/
void USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    if (pUSARTHandle->TxBusyState != USART_BUSY_IN_TX)
    {
        pUSARTHandle->pTxBuffer = pTxBuffer;
        pUSARTHandle->TxLen = Len;
        pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

        /* Enable TXE interrupt */
        pUSARTHandle->pUSARTx->CR1 |= USART_CR1_TXEIE;
    }
}

/*********************************************************************
 * 		     		  - USART_ReceiveDataIT (Interrupt RX)
 **********************************************************************/
void USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    if (pUSARTHandle->RxBusyState != USART_BUSY_IN_RX)
    {
        pUSARTHandle->pRxBuffer = pRxBuffer;
        pUSARTHandle->RxLen = Len;
        pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

        /* Enable RXNE interrupt */
        pUSARTHandle->pUSARTx->CR1 |= USART_CR1_RXNEIE;
    }
}

/*********************************************************************
 * 		     		  - USART_IRQInterruptConfig
 **********************************************************************/
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
        NVIC_EnableIRQ(IRQNumber);
    else
        NVIC_DisableIRQ(IRQNumber);
}

/*********************************************************************
 * 		     		  - USART_IRQPriorityConfig
 **********************************************************************/
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    NVIC_SetPriority(IRQNumber, IRQPriority);
}

/*********************************************************************
 *     			  - USART_IRQHandling
 **********************************************************************/
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
    USART_TypeDef *pUSARTx = pUSARTHandle->pUSARTx;

    uint32_t temp1, temp2;

    /********************* TXE INTERRUPT *********************/
    temp1 = pUSARTx->ISR & USART_ISR_TXE;
    temp2 = pUSARTx->CR1 & USART_CR1_TXEIE;

    if (temp1 && temp2)
    {
        if (pUSARTHandle->TxLen > 0)
        {
            pUSARTx->TDR = *(pUSARTHandle->pTxBuffer) & 0xFF;
            pUSARTHandle->pTxBuffer++;
            pUSARTHandle->TxLen--;
        }
        else
        {
            /* Disable TXEIE, enable TCIE */
            pUSARTx->CR1 &= ~USART_CR1_TXEIE;
            pUSARTx->CR1 |= USART_CR1_TCIE;
        }
    }

    /********************* TC INTERRUPT *********************/
    temp1 = pUSARTx->ISR & USART_ISR_TC;
    temp2 = pUSARTx->CR1 & USART_CR1_TCIE;

    if (temp1 && temp2)
    {
        pUSARTx->CR1 &= ~USART_CR1_TCIE;
        pUSARTHandle->TxBusyState = USART_READY;
    }

    /********************* RXNE INTERRUPT *********************/
    temp1 = pUSARTx->ISR & USART_ISR_RXNE;
    temp2 = pUSARTx->CR1 & USART_CR1_RXNEIE;

    if (temp1 && temp2)
    {
        *(pUSARTHandle->pRxBuffer) = pUSARTx->RDR & 0xFF;
        pUSARTHandle->pRxBuffer++;
        pUSARTHandle->RxLen--;

        if (pUSARTHandle->RxLen == 0)
        {
            pUSARTx->CR1 &= ~USART_CR1_RXNEIE;
            pUSARTHandle->RxBusyState = USART_READY;
        }
    }
}

