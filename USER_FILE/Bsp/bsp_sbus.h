/*
 * 本文件属于 Bsp 硬件层，定义 UART5 SBUS 接收接口和接收统计。
 * 依赖 STM32 HAL、CubeMX 生成的 huart5；上层通过 bsp_sbus_read 取得完整帧。
 * DMA 缓冲固定放在 D2 SRAM1，避开 DMA1 无法访问的 DTCM；实现见 bsp_sbus.c。
 */
#ifndef BSP_SBUS_H
#define BSP_SBUS_H

#include <stdint.h>

#ifndef BSP_SBUS_DMA_ADDR
#define BSP_SBUS_DMA_ADDR 0x30000000u
#endif
#ifndef BSP_SBUS_DMA_LEN
#define BSP_SBUS_DMA_LEN 32u
#endif

/* bsp_sbus_init：检查 UART5 波特率并启动 To-Idle DMA 接收
 * 返回：0 成功；-1 波特率不是 100000（请在 CubeMX 修改）；-2 启动接收失败。
 */
int bsp_sbus_init(void);

/* bsp_sbus_read：拷贝当前完整帧
 * 返回：有效帧序号；0 表示尚未收到有效帧，回绕时跳过 0。
 * 说明：只供优先级高于 UART5/DMA 的 TIM6 调用，不适用于主循环并发读取。
 */
uint32_t bsp_sbus_read(uint8_t out[25]);

/* bsp_sbus_stats：读取坏帧和 UART 错误计数。 */
void bsp_sbus_stats(uint32_t *bad, uint32_t *err);

#endif
