/*
 * 本文件属于 Componnet 算法组件层，实现与硬件无关的 SBUS 解析。
 * 仅依赖自身头文件及 <stdint.h>，无 HAL 或 App 依赖。
 * SBUS 数据区是连续的 16 个 11 位小端通道；帧头为 0x0F，帧尾为 0x00。
 * 中位为 1024，满量程约为 353~1695；标志字节 bit2 为单帧丢失，bit3 为 failsafe。
 */
#include "sbus.h"

/* sbus_decode：验证帧边界后按位流解码 16 个 11 位通道。 */
int sbus_decode(const uint8_t f[SBUS_FRAME_LEN], sbus_frame_t *out) {
    sbus_frame_t decoded;
    if (f == 0 || out == 0 || f[0] != 0x0fu || f[SBUS_FRAME_LEN - 1u] != 0x00u)
        return -1;

    for (uint8_t i = 0u; i < SBUS_CH_NUM; i++) {
        uint16_t value = 0u;
        for (uint8_t b = 0u; b < 11u; b++) {
            const uint16_t bit = (uint16_t)(i * 11u + b);
            value |= (uint16_t)(((f[1u + bit / 8u] >> (bit % 8u)) & 1u) << b);
        }
        decoded.ch[i] = value;
    }
    decoded.lost = (uint8_t)((f[23] >> 2u) & 0x01u);
    decoded.failsafe = (uint8_t)((f[23] >> 3u) & 0x01u);
    *out = decoded;
    return 0;
}

/* sbus_norm：使用 SBUS 中位和满量程常数进行归一化、限幅和死区处理。 */
float sbus_norm(uint16_t raw, float deadband) {
    float value = ((float)raw - 1024.0f) / 671.0f;
    if (value > 1.0f)
        value = 1.0f;
    else if (value < -1.0f)
        value = -1.0f;
    if (value < 0.0f ? -value < deadband : value < deadband)
        return 0.0f;
    return value;
}

/* sbus_switch3：按 ET07 三挡开关的原始值划分低、中、高档。 */
uint8_t sbus_switch3(uint16_t raw) {
    if (raw < 700u)
        return 0u;
    if (raw < 1350u)
        return 1u;
    return 2u;
}
