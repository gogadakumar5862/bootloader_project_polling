#include "ymodem.h"
#include "stm32f0xx_usart_driver.h"

extern USART_Handle_t usart1Handle;

#define SOH 0x01
#define STX 0x02
#define ACK 0x06
#define CRC_REQ 'C'

void ymodem_init(ymodem_ctx_t *ctx, uint8_t *dst, uint32_t max_size)
{
    ctx->state = YMODEM_IDLE;
    ctx->dst = dst;
    ctx->max_size = max_size;
    ctx->offset = 0;
}

int ymodem_process(ymodem_ctx_t *ctx)
{
    if (usart1Handle.RxBusyState != USART_READY)
        return 0;

    uint8_t c = *(usart1Handle.pRxBuffer);

    switch (ctx->state)
    {
        case YMODEM_IDLE:
            USART_SendData(&usart1Handle, (uint8_t*)CRC_REQ, 1);
            ctx->state = YMODEM_WAIT_START;
            break;

        case YMODEM_WAIT_START:
            if (c == SOH || c == STX)
            {
                ctx->index = 0;
                ctx->packet[ctx->index++] = c;
                ctx->state = YMODEM_RECV_PACKET;
            }
            break;

        case YMODEM_RECV_PACKET:
            ctx->packet[ctx->index++] = c;
            if (ctx->index >= 133)
            {
                USART_SendData(&usart1Handle, (uint8_t*)ACK, 1);
                ctx->state = YMODEM_DONE;
            }
            break;

        case YMODEM_DONE:
            return 1;
    }

    /* Re-arm RX interrupt */
    USART_ReceiveDataIT(&usart1Handle, usart1Handle.pRxBuffer, 1);

    return 0;
}

