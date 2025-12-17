#ifndef YMODEM_H
#define YMODEM_H

#include <stdint.h>

/* 1️⃣ DEFINE ENUM FIRST */
typedef enum {
    YMODEM_IDLE,
    YMODEM_WAIT_START,
    YMODEM_RECV_PACKET,
    YMODEM_EOT,
    YMODEM_DONE,
    YMODEM_ERROR
} ymodem_state_t;

/* 2️⃣ THEN USE IT IN STRUCT */
typedef struct {
    ymodem_state_t state;
    uint8_t *dst;
    uint32_t max_size;
    uint32_t offset;
    uint8_t packet[1031];
    uint32_t index;
    uint8_t expected_blk;
} ymodem_ctx_t;

void ymodem_init(ymodem_ctx_t *ctx, uint8_t *dst, uint32_t max_size);
int  ymodem_process(ymodem_ctx_t *ctx);

#endif

