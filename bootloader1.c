#include "stm32f0xx.h"
#include "stm32f0xx_gpio_driver.h"
#include "stm32f0xx_rcc_driver.h"
#include "stm32f0xx_led_driver.h"
#include <stdint.h>

/* ============================================================
   BOOTLOADER CONFIG
   ============================================================ */
#define RAM_APP_ADDR        0x20000000
#define MAX_FIRMWARE_SIZE  (4 * 1024U)

/* ============================================================
   UART DRIVER - POLLING
   ============================================================ */

/* USART Flags */
#define USART_FLAG_TXE   (1U << 7)
#define USART_FLAG_RXNE  (1U << 5)
#define USART_FLAG_TC    (1U << 6)

/* USART Configuration */
typedef struct {
    uint8_t  USART_Mode;
    uint32_t USART_Baud;
    uint8_t  USART_NoOfStopBits;
    uint8_t  USART_WordLength;
    uint8_t  USART_ParityControl;
} USART_Config_t;

/* USART Handle */
typedef struct {
    USART_TypeDef   *pUSARTx;
    USART_Config_t   USART_Config;
} USART_Handle_t;

/* Global handle (used by main.c) */
USART_Handle_t usart1Handle;

/* ============================================================
   UART LOW LEVEL FUNCTIONS
   ============================================================ */

void USART_PeriClockControl(USART_TypeDef *pUSARTx, uint8_t EnOrDi)
{
    RCC_PClkControl_USART(pUSARTx, EnOrDi);
}

static uint32_t USART_ComputeBRR(uint32_t pclk, uint32_t baud)
{
    if (baud == 0) return 0;
    return (pclk + (baud / 2U)) / baud;
}

uint8_t USART_GetFlagStatus(USART_TypeDef *pUSARTx, uint32_t FlagName)
{
    return (pUSARTx->ISR & FlagName) ? 1U : 0U;
}

void USART_Init(USART_Handle_t *pUSARTHandle)
{
    USART_TypeDef *pUSARTx = pUSARTHandle->pUSARTx;
    USART_Config_t *cfg = &pUSARTHandle->USART_Config;

    USART_PeriClockControl(pUSARTx, ENABLE);

    pUSARTx->CR1 &= ~USART_CR1_UE;

    /* Word length */
    if (cfg->USART_WordLength == 9)
        pUSARTx->CR1 |= (1U << 12);
    else
        pUSARTx->CR1 &= ~(1U << 12);

    /* Parity */
    if (cfg->USART_ParityControl == 0)
    {
        pUSARTx->CR1 &= ~USART_CR1_PCE;
    }
    else
    {
        pUSARTx->CR1 |= USART_CR1_PCE;
        if (cfg->USART_ParityControl == 2)
            pUSARTx->CR1 |= (1U << 9);
        else
            pUSARTx->CR1 &= ~(1U << 9);
    }

    /* Stop bits */
    if (cfg->USART_NoOfStopBits == 2)
        pUSARTx->CR2 |= (0x2U << 12);
    else
        pUSARTx->CR2 &= ~(0x3U << 12);

    uint32_t pclk = RCC_GetHCLKValue();
    pUSARTx->BRR = USART_ComputeBRR(pclk, cfg->USART_Baud);

    pUSARTx->CR1 |= USART_CR1_TE | USART_CR1_RE;
    pUSARTx->CR1 |= USART_CR1_UE;
}

/* ============================================================
   UART POLLING TX / RX
   ============================================================ */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle,
                       uint8_t *pRxBuffer,
                       uint32_t Len)
{
    USART_TypeDef *pUSARTx = pUSARTHandle->pUSARTx;

    for (uint32_t i = 0; i < Len; i++)
    {
        while (!USART_GetFlagStatus(pUSARTx, USART_FLAG_RXNE));
        pRxBuffer[i] = (uint8_t)(pUSARTx->RDR & 0xFF);
    }
}

void USART_SendData(USART_Handle_t *pUSARTHandle,
                    uint8_t *pTxBuffer,
                    uint32_t Len)
{
    USART_TypeDef *pUSARTx = pUSARTHandle->pUSARTx;

    for (uint32_t i = 0; i < Len; i++)
    {
        while (!USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE));
        pUSARTx->TDR = pTxBuffer[i];
    }

    while (!USART_GetFlagStatus(pUSARTx, USART_FLAG_TC));
}

/* ============================================================
   UART HELPER FUNCTIONS (USED BY YMODEM)
   ============================================================ */

void uart_putc(uint8_t c)
{
    USART_SendData(&usart1Handle, &c, 1);
}

int uart_getc_timeout(uint8_t *c, uint32_t timeout)
{
    USART_TypeDef *u = usart1Handle.pUSARTx;

    while (timeout--)
    {
        if (USART_GetFlagStatus(u, USART_FLAG_RXNE))
        {
            *c = (uint8_t)(u->RDR & 0xFF);
            return 1;
        }
    }
    return 0;
}

void uart_puts(const char *s)
{
    while (*s)
        uart_putc((uint8_t)*s++);
}

static void delay_cycles(volatile uint32_t n)
{
    while (n--) __NOP();
}

/* ============================================================
   YMODEM IMPLEMENTATION (UNCHANGED)
   ============================================================ */

#define SOH 0x01
#define STX 0x02
#define EOT 0x04
#define ACK 0x06
#define NAK 0x15
#define CAN 0x18
#define CRC_REQ 'C'

#define PACKET_SIZE     128
#define PACKET_1K_SIZE  1024
#define MAX_RETRIES     10

static uint16_t crc16(const uint8_t *buf, int len)
{
    uint16_t crc = 0;
    while (len--)
    {
        crc ^= (*buf++) << 8;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000) ?
                  (crc << 1) ^ 0x1021 :
                  (crc << 1);
    }
    return crc;
}

int ymodem_receive_to_ram(uint8_t *dst_addr,
                          uint32_t max_size,
                          uint32_t *received_size)
{
    uint8_t packet[1031];
    uint32_t offset = 0;
    uint8_t expected_blk = 1;
    uint32_t retries = 0;

    uart_putc(CRC_REQ);

    while (1)
    {
        uint8_t first;

        if (!uart_getc_timeout(&first, 200000))
        {
            if (++retries > MAX_RETRIES)
                return -1;
            uart_putc(CRC_REQ);
            continue;
        }

        retries = 0;

        if (first == SOH || first == STX)
        {
            uint32_t block_len =
                (first == SOH) ? PACKET_SIZE : PACKET_1K_SIZE;

            for (uint32_t i = 0; i < block_len + 4; i++)
                uart_getc_timeout(&packet[i], 100000);

            uint8_t blk = packet[0];
            uint8_t inv = packet[1];

            if ((blk + inv) != 0xFF)
            {
                uart_putc(NAK);
                continue;
            }

            uint16_t rx_crc =
                (packet[block_len + 2] << 8) |
                 packet[block_len + 3];

            uint16_t calc_crc =
                crc16(&packet[2], block_len);

            if (rx_crc != calc_crc)
            {
                uart_putc(NAK);
                continue;
            }

            if (blk == expected_blk)
            {
                for (uint32_t i = 0;
                     i < block_len && offset < max_size;
                     i++)
                {
                    dst_addr[offset++] = packet[2 + i];
                }
                expected_blk++;
            }

            uart_putc(ACK);
        }
        else if (first == EOT)
        {
            uart_putc(NAK);
            uart_putc(ACK);
            break;
        }
        else if (first == CAN)
        {
            return -3;
        }
    }

    if (received_size)
        *received_size = offset;

    return 0;
}

