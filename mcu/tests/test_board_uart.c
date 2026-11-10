#include "protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#define RX_RING_SIZE 128u
#define HAL_OK 0
#define HAL_UART_STATE_READY 1

typedef struct {
    int RxState;
} UART_HandleTypeDef;

static UART_HandleTypeDef uart = {HAL_UART_STATE_READY};
static Control control;
static Parser parser;
static Reply pending_reply;
static uint8_t receive_byte;
static volatile uint8_t receive_ring[RX_RING_SIZE];
static volatile unsigned receive_head, receive_tail;
static volatile uint32_t last_receive_ms;
static volatile bool receive_error, transmitting;
static char transmit_line[REPLY_CAPACITY];
static uint32_t clock_ms;
static unsigned sent;
static char last_sent[REPLY_CAPACITY];

static uint32_t HAL_GetTick(void)
{
    return clock_ms;
}

static int HAL_UART_Receive_IT(UART_HandleTypeDef *handle, uint8_t *buffer, uint16_t size)
{
    assert(handle == &uart && buffer == &receive_byte && size == 1);
    return HAL_OK;
}

static int HAL_UART_Transmit_IT(UART_HandleTypeDef *handle, uint8_t *buffer, uint16_t size)
{
    assert(handle == &uart && buffer == (uint8_t *)transmit_line && size < REPLY_CAPACITY);
    memcpy(last_sent, buffer, size);
    last_sent[size] = '\0';
    ++sent;
    return HAL_OK;
}

#include "board_uart.inc"

static void deliver(const char *text, uint32_t now)
{
    clock_ms = now;
    for (; *text; ++text) {
        receive_byte = (uint8_t)*text;
        HAL_UART_RxCpltCallback(&uart);
    }
}

int main(void)
{
    control_init(&control, 1);
    deliver("CMD,0.", 100);
    uart_poll();
    assert(sent == 0 && !control.command_active);
    deliver("2,0\r\n", 105);
    uart_poll();
    assert(control.command_active && sent == 0);
    clock_ms = 106;
    uart_poll();
    assert(sent == 0);
    clock_ms = 107;
    uart_poll();
    assert(sent == 1 && transmitting && strcmp(last_sent, "ACK,CMD\n") == 0);
    HAL_UART_ErrorCallback(&uart);  /* 回包时收到本机回波的错误。 */
    HAL_UART_TxCpltCallback(&uart);
    uart_poll();
    assert(sent == 1 && !receive_error);
    puts("PASS real board UART poll: split request, one CRLF ACK and 2 ms quiet");

    deliver("STOP", 110);
    uart_poll();
    deliver("\n", 111);
    HAL_UART_ErrorCallback(&uart);
    uart_poll();
    clock_ms = 113;
    uart_poll();
    assert(!control.stop_latched && strcmp(last_sent, "ERR,INPUT\n") == 0);
    HAL_UART_TxCpltCallback(&uart);
    deliver("STOP\n", 115);
    uart_poll();
    clock_ms = 117;
    uart_poll();
    assert(control.stop_latched && strcmp(last_sent, "ACK,STOP\n") == 0);
    puts("PASS real board callbacks discard damaged request and accept the next complete one");
    return 0;
}
