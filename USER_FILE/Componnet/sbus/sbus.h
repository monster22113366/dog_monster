/*
 * 本文件属于 Componnet 算法组件层，实现 SBUS 帧解码和通道值映射。
 * 只依赖 <stdint.h>，与 UART、DMA 及其他硬件无关，可直接在 PC 上编译测试。
 * SBUS 通道中位为 1024，满量程约为 353~1695；原始通道下标从 0 开始。
 */
#ifndef COMPONENT_SBUS_H
#define COMPONENT_SBUS_H

#include <stdint.h>

#define SBUS_FRAME_LEN 25u
#define SBUS_CH_NUM 16u

typedef struct {
    uint16_t ch[SBUS_CH_NUM];
    uint8_t lost;
    uint8_t failsafe;
} sbus_frame_t;

/* sbus_decode：校验并解码一个完整 SBUS 帧
 * 参数：f 为 25 字节帧；out 为输出结构。
 * 返回：0 成功；-1 帧头、帧尾、参数或格式校验失败。
 * 说明：失败时不写 out；通道按 11 位小端位流解码。
 */
int sbus_decode(const uint8_t f[SBUS_FRAME_LEN], sbus_frame_t *out);

/* sbus_norm：把 SBUS 原始通道映射到 [-1, 1] 并应用死区
 * 参数：raw 为原始通道值；deadband 为归一化死区。
 * 返回：限幅后的归一化值；死区内返回 0。
 */
float sbus_norm(uint16_t raw, float deadband);

/* sbus_switch3：把三挡开关原始值映射为低、中、高 0/1/2。 */
uint8_t sbus_switch3(uint16_t raw);

#endif
