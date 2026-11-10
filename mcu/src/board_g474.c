#include "stm32g4xx_hal.h"
#include "drive.h"
#include "protocol.h"
#include <errno.h>

#define RX_RING_SIZE 128u

static FDCAN_HandleTypeDef can;
static UART_HandleTypeDef uart;
static Control control;
static Drive drive;
static Parser parser;
static Reply pending_reply;
static uint8_t receive_byte;
static volatile uint8_t receive_ring[RX_RING_SIZE];
static volatile unsigned receive_head, receive_tail;
static volatile uint32_t last_receive_ms;
static volatile bool receive_error, transmitting;
static char transmit_line[REPLY_CAPACITY];

void _init(void) {}
void _fini(void) {}

void *_sbrk(ptrdiff_t increment)
{
    extern char _ebss, _estack;
    static uintptr_t heap;
    const uintptr_t first = (uintptr_t)&_ebss;
    const uintptr_t limit = (uintptr_t)&_estack - 4096u;
    if (heap == 0) {
        heap = first;
    }
    /* newlib 的 strtod 可分配临时空间，保留链接脚本约定的 4 KiB 栈。 */
    if (increment < -(ptrdiff_t)(heap - first) || increment > (ptrdiff_t)(limit - heap)) {
        errno = ENOMEM;
        return (void *)-1;
    }
    void *previous = (void *)heap;
    heap += increment;
    return previous;
}

static void check(HAL_StatusTypeDef status)
{
    if (status != HAL_OK) {
        for (;;) {}
    }
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void LPUART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&uart);
}

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_DisableUCPDDeadBattery();
}

static void clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
    osc.HSIState = RCC_HSI_ON;
    osc.HSI48State = RCC_HSI48_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_NONE;
    check(HAL_RCC_OscConfig(&osc));

    RCC_ClkInitTypeDef clock = {0};
    clock.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clock.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clock.APB1CLKDivider = RCC_HCLK_DIV1;
    clock.APB2CLKDivider = RCC_HCLK_DIV1;
    check(HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_0));
}

static uint64_t boot_identity(void)
{
    __HAL_RCC_RNG_CONFIG(RCC_RNGCLKSOURCE_HSI48);
    __HAL_RCC_RNG_CLK_ENABLE();
    RNG_HandleTypeDef rng = {0};
    rng.Instance = RNG;
    uint32_t high, low;
    check(HAL_RNG_Init(&rng));
    do {
        check(HAL_RNG_GenerateRandomNumber(&rng, &high));
        check(HAL_RNG_GenerateRandomNumber(&rng, &low));
    } while (high == 0 && low == 0);
    return ((uint64_t)high << 32) | low;
}

void HAL_UART_MspInit(UART_HandleTypeDef *handle)
{
    if (handle->Instance != LPUART1) {
        return;
    }
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_LPUART1_CONFIG(RCC_LPUART1CLKSOURCE_HSI);
    __HAL_RCC_LPUART1_CLK_ENABLE();
    __HAL_RCC_LPUART1_FORCE_RESET();
    __HAL_RCC_LPUART1_RELEASE_RESET();

    GPIO_InitTypeDef pins = {0};
    pins.Mode = GPIO_MODE_AF_PP;
    pins.Speed = GPIO_SPEED_FREQ_HIGH;
    pins.Alternate = GPIO_AF8_LPUART1;
    pins.Pin = GPIO_PIN_1;
    pins.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &pins);
    pins.Pin = GPIO_PIN_0;
    pins.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &pins);

    /* 启用 UART 复用前，先保持 RS485 方向脚在接收态。 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    pins.Pin = GPIO_PIN_1;
    pins.Mode = GPIO_MODE_OUTPUT_PP;
    pins.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &pins);
}

static void uart_init(void)
{
    uart.Instance = LPUART1;
    uart.Init.BaudRate = 115200;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.Parity = UART_PARITY_NONE;
    uart.Init.Mode = UART_MODE_TX_RX;
    uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart.Init.OverSampling = UART_OVERSAMPLING_16;
    uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    check(HAL_RS485Ex_Init(&uart, UART_DE_POLARITY_HIGH, 2, 2));

    GPIO_InitTypeDef direction = {0};
    direction.Pin = GPIO_PIN_1;
    direction.Mode = GPIO_MODE_AF_PP;
    direction.Speed = GPIO_SPEED_FREQ_HIGH;
    direction.Alternate = GPIO_AF12_LPUART1;
    HAL_GPIO_Init(GPIOB, &direction);
    HAL_NVIC_SetPriority(LPUART1_IRQn, 5, 0);
    check(HAL_UART_Receive_IT(&uart, &receive_byte, 1));
    HAL_NVIC_ClearPendingIRQ(LPUART1_IRQn);
    HAL_NVIC_EnableIRQ(LPUART1_IRQn);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *handle)
{
    if (handle != &uart) {
        return;
    }
    if (!transmitting) {
        last_receive_ms = HAL_GetTick();
        unsigned next = (receive_head + 1u) % RX_RING_SIZE;
        if (next == receive_tail) {
            receive_error = true;
        } else {
            receive_ring[receive_head] = receive_byte;
            receive_head = next;
        }
    }
    if (HAL_UART_Receive_IT(&uart, &receive_byte, 1) != HAL_OK) {
        receive_error = true;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *handle)
{
    if (handle == &uart) {
        transmitting = false;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle)
{
    if (handle == &uart) {
        if (!transmitting) {
            receive_error = true;
        }
        if (handle->RxState == HAL_UART_STATE_READY) {
            (void)HAL_UART_Receive_IT(handle, &receive_byte, 1);
        }
    }
}

static void uart_poll(void)
{
    while (pending_reply == REPLY_NONE && receive_tail != receive_head) {
        char byte = (char)receive_ring[receive_tail];
        receive_tail = (receive_tail + 1u) % RX_RING_SIZE;
        if (receive_error) {
            parser.discard = true;
            receive_error = false;
        }
        pending_reply = protocol_byte(&parser, &control, byte, HAL_GetTick());
    }
    uint32_t now = HAL_GetTick();
    if (pending_reply != REPLY_NONE && !transmitting && now - last_receive_ms >= REPLY_QUIET_MS) {
        size_t count = protocol_reply(transmit_line, pending_reply, &control, now);
        transmitting = true;
        if (HAL_UART_Transmit_IT(&uart, (uint8_t *)transmit_line, (uint16_t)count) != HAL_OK) {
            transmitting = false;
        }
        pending_reply = REPLY_NONE;
    }
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *handle)
{
    if (handle->Instance != FDCAN3) {
        return;
    }
    RCC_PeriphCLKInitTypeDef clock = {0};
    clock.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    clock.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    check(HAL_RCCEx_PeriphCLKConfig(&clock));
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_FDCAN_FORCE_RESET();
    __HAL_RCC_FDCAN_RELEASE_RESET();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef pins = {0};
    pins.Pin = GPIO_PIN_3 | GPIO_PIN_4;
    pins.Mode = GPIO_MODE_AF_PP;
    pins.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    pins.Alternate = GPIO_AF11_FDCAN3;
    HAL_GPIO_Init(GPIOB, &pins);
}

static void can_init(void)
{
    can.Instance = FDCAN3;
    can.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    can.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    can.Init.Mode = FDCAN_MODE_NORMAL;
    can.Init.AutoRetransmission = DISABLE;
    can.Init.NominalPrescaler = 2;
    can.Init.NominalSyncJumpWidth = 1;
    can.Init.NominalTimeSeg1 = 13;
    can.Init.NominalTimeSeg2 = 2;
    can.Init.DataPrescaler = 1;
    can.Init.DataSyncJumpWidth = 1;
    can.Init.DataTimeSeg1 = 1;
    can.Init.DataTimeSeg2 = 1;
    can.Init.StdFiltersNbr = 1;
    can.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    check(HAL_FDCAN_Init(&can));

    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x581;
    filter.FilterID2 = 0x7ff;
    check(HAL_FDCAN_ConfigFilter(&can, &filter));
    check(HAL_FDCAN_ConfigGlobalFilter(&can, FDCAN_REJECT, FDCAN_REJECT,
                                      FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE));
    check(HAL_FDCAN_Start(&can));
    check(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN) == 16000000u ? HAL_OK : HAL_ERROR);
}

static bool can_send(void *user, const uint8_t frame[8])
{
    (void)user;
    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = 0x601;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(&can, &header, (uint8_t *)frame) == HAL_OK;
}

static void can_poll(void)
{
    while (HAL_FDCAN_GetRxFifoFillLevel(&can, FDCAN_RX_FIFO0) > 0) {
        FDCAN_RxHeaderTypeDef header;
        uint8_t bytes[64];
        if (HAL_FDCAN_GetRxMessage(&can, FDCAN_RX_FIFO0, &header, bytes) != HAL_OK) {
            break;
        }
        if (header.IdType == FDCAN_STANDARD_ID && header.Identifier == 0x581 &&
            header.RxFrameType == FDCAN_DATA_FRAME && header.DataLength == FDCAN_DLC_BYTES_8) {
            drive_receive(&drive, &control, bytes, HAL_GetTick());
        }
    }
    drive_poll(&drive, &control, (can.Instance->PSR & FDCAN_PSR_BO) != 0, HAL_GetTick());
}

int main(void)
{
    __disable_irq();
    SCB->VTOR = FLASH_BASE;
    SysTick->CTRL = 0;
    for (unsigned i = 0; i < 8; ++i) {
        NVIC->ICER[i] = 0xffffffffu;
        NVIC->ICPR[i] = 0xffffffffu;
    }
    __DSB();
    __ISB();
    HAL_Init();
    __enable_irq();
    clock_init();
    can_init();
    control_init(&control, boot_identity());
    drive_init(&drive, can_send, NULL, HAL_GetTick());
    uart_init();

    for (;;) {
        uart_poll();
        can_poll();
        HAL_Delay(1);
    }
}
