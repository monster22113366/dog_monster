/* STM32 FDCAN 硬件接口：只描述总线和 CAN 帧，不包含上层协议。 */
#ifndef BSP_CAN_H
#define BSP_CAN_H
#include <stdint.h>

enum { BSP_CAN_BUS1 = 0, BSP_CAN_BUS2 = 1, BSP_CAN_BUS3 = 2 };
#define BSP_CAN_BUS_NUM 3u
#ifndef BSP_CAN3_TDC_OFFSET
#define BSP_CAN3_TDC_OFFSET 12u
#endif
#ifndef BSP_CAN3_TDC_FILTER
#define BSP_CAN3_TDC_FILTER 0u
#endif

typedef struct {
    uint32_t id;
    uint8_t len;
    uint8_t data[64];
    uint8_t extended, fd, brs;
} bsp_can_frame_t;

typedef void (*bsp_can_rx_fn)(uint8_t bus, const bsp_can_frame_t *frame);

typedef struct {
    uint32_t rx;        /* 从硬件 FIFO 成功取出的帧数，包含被拒绝的帧。 */
    uint32_t rx_reject; /* 不符合硬件帧约束的接收帧数。 */
    uint32_t tx;        /* 成功加入发送 FIFO 的帧数。 */
    uint32_t tx_drop;   /* 发送 FIFO 满的请求数。 */
    uint32_t tx_error;  /* HAL 入队失败次数；非法 API 参数不计入硬件统计。 */
    uint32_t bus_off;   /* bus-off 通知次数。 */
} bsp_can_stats_t;

/* 启动三路总线；CubeMX 的 MX_FDCANx_Init 须已完成。 */
void bsp_can_init(void);
/* 非阻塞发送：1 入队，0 FIFO 满，-1 参数或 HAL 错误。 */
int bsp_can_send(uint8_t bus, const bsp_can_frame_t *frame);
/* 每路一个接收者，在启动时登记；NULL 表示不投递。回调须同步消费帧。 */
void bsp_can_set_rx_callback(uint8_t bus, bsp_can_rx_fn callback);
/* 无效总线返回 NULL；计数在中断和发送路径更新。 */
const bsp_can_stats_t *bsp_can_stats(uint8_t bus);
#endif
