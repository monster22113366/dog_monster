/*
 * 本文件属于 Bsp 硬件层，实现 UART5 的 SBUS To-Idle DMA 接收。
 * DMA 缓冲位于 D2 SRAM1（0x30000000）；DMA1 不能访问 DTCM，故不使用普通静态数组。
 * 当前工程未开启 D-Cache，无需维护；将来开启 D-Cache 时必须用 MPU 把该区配置为 non-cacheable。
 * UART5 应在 CubeMX 配置为 100000 波特率、9 位字长（含偶校验）、2 停止位、RX only。
 */
#include "bsp_sbus.h"
#include "main.h"
#include "usart.h"
#include <string.h>

static uint8_t s_frame[2][25];
static uint32_t s_frame_seq[2];
static volatile uint8_t s_active;
static volatile uint32_t s_seq;
static volatile uint32_t s_bad_count;
static volatile uint32_t s_err_count;

_Static_assert(BSP_SBUS_DMA_LEN > 25u && BSP_SBUS_DMA_LEN <= UINT16_MAX,
               "DMA buffer must detect oversized frames and fit HAL length");

/* bsp_sbus_dma_buf：返回不进入链接脚本的 D2 SRAM1 DMA 缓冲地址。 */
static uint8_t *bsp_sbus_dma_buf(void) {
    return (uint8_t *)BSP_SBUS_DMA_ADDR;
}

/* bsp_sbus_restart：重新开始固定长度 To-Idle DMA，并关闭半传输中断。
 * 关闭半传输是因为 SBUS 半帧事件不是完整帧，不能交给上层当作接收结果。
 */
static int bsp_sbus_restart(void) {
    const HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart5, bsp_sbus_dma_buf(), BSP_SBUS_DMA_LEN);
    if (huart5.hdmarx != 0)
        __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    return status == HAL_OK ? 0 : -1;
}

/* bsp_sbus_init：复位软件状态、打开 D2 SRAM1 时钟并检查 CubeMX 波特率。 */
int bsp_sbus_init(void) {
    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    s_active = 0u;
    s_seq = 0u;
    s_bad_count = 0u;
    s_err_count = 0u;
    memset(s_frame, 0, sizeof(s_frame));
    memset(s_frame_seq, 0, sizeof(s_frame_seq));
    if (huart5.Init.BaudRate != 100000u)
        return -1; /* 请在 CubeMX 把 UART5 波特率改为 100000，驱动不静默改写。 */
    return bsp_sbus_restart() == 0 ? 0 : -2;
}

/* HAL_UARTEx_RxEventCallback：只接收恰好 25 字节且边界正确的 SBUS 帧。
 * 写者只写非活动缓冲，完成后切换活动下标并递增序号；TIM6 优先级高于 UART/DMA，
 * 读者可以打断写者但写者不能打断读者，因此读者复制活动缓冲时必然得到完整帧。
 * 每份缓冲携带对应序号，避免在切换下标与 s_seq 更新之间被抢占时帧与序号不一致。
 * DMB 同时约束编译器与 CPU 的发布顺序；UART5 与 DMA 同为优先级 6，不互相抢占。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == 0 || huart->Instance != UART5)
        return;
    if (Size == 25u && bsp_sbus_dma_buf()[0] == 0x0fu && bsp_sbus_dma_buf()[24] == 0x00u) {
        const uint8_t next = (uint8_t)(s_active ^ 1u);
        memcpy(s_frame[next], bsp_sbus_dma_buf(), 25u);
        s_frame_seq[next] = s_seq == UINT32_MAX ? 1u : s_seq + 1u;
        __DMB();
        s_active = next;
        s_seq = s_frame_seq[next];
    } else {
        s_bad_count++;
    }
    (void)bsp_sbus_restart();
}

/* HAL_UART_ErrorCallback：只处理 UART5，统计错误后恢复 To-Idle DMA 接收。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == 0 || huart->Instance != UART5)
        return;
    s_err_count++;
    (void)bsp_sbus_restart();
}

/* bsp_sbus_read：在高优先级 TIM6 上下文中复制活动缓冲并返回其序号。 */
uint32_t bsp_sbus_read(uint8_t out[25]) {
    const uint8_t active = s_active;
    __DMB();
    if (out == 0)
        return s_frame_seq[active];
    memcpy(out, s_frame[active], 25u);
    return s_frame_seq[active];
}

/* bsp_sbus_stats：复制接收统计，允许调用者只关心其中一项。 */
void bsp_sbus_stats(uint32_t *bad, uint32_t *err) {
    if (bad != 0)
        *bad = s_bad_count;
    if (err != 0)
        *err = s_err_count;
}
