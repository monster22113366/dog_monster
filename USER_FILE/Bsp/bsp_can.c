/* 通用 FDCAN 驱动：FIFO 收发、帧属性转换、TDC 和 bus-off 恢复。 */
#include "bsp_can.h"
#include "fdcan.h"

/* ============================================================================
 * FDCAN 总线配置
 * ========================================================================== */
static const struct {
    FDCAN_HandleTypeDef *handle;
    uint8_t fd;
} s_bus[BSP_CAN_BUS_NUM] = {
    {&hfdcan1, 0u}, {&hfdcan2, 0u}, {&hfdcan3, 1u}
};
static bsp_can_stats_t s_stats[BSP_CAN_BUS_NUM];
static bsp_can_rx_fn s_rx[BSP_CAN_BUS_NUM];
/* 当前 HAL 的 DataLength 为 0..15 DLC 编码，不是字节数。 */
static const uint8_t s_lengths[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

/* ============================================================================
 * 内部辅助函数：硬件句柄定位
 * ========================================================================== */
static uint8_t bus_of(FDCAN_HandleTypeDef *handle) {
    for (uint8_t bus = 0u; bus < BSP_CAN_BUS_NUM; ++bus)
        if (s_bus[bus].handle == handle) return bus;
    return BSP_CAN_BUS_NUM;
}

/* ============================================================================
 * 核心接口：CAN 初始化与接收者登记
 * ========================================================================== */
void bsp_can_set_rx_callback(uint8_t bus, bsp_can_rx_fn callback) {
    if (bus < BSP_CAN_BUS_NUM) s_rx[bus] = callback;
}

/* 保持全接收过滤、拒绝远程帧及原有通知；第三路保留 TDC。 */
void bsp_can_init(void) {
    for (uint8_t bus = 0u; bus < BSP_CAN_BUS_NUM; ++bus) {
        FDCAN_HandleTypeDef *h = s_bus[bus].handle;
        if (HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                        FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
            Error_Handler();
        if (s_bus[bus].fd) {
            if (HAL_FDCAN_ConfigTxDelayCompensation(h, BSP_CAN3_TDC_OFFSET, BSP_CAN3_TDC_FILTER) != HAL_OK ||
                HAL_FDCAN_EnableTxDelayCompensation(h) != HAL_OK)
                Error_Handler();
        }
        if (HAL_FDCAN_ActivateNotification(h, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0u) != HAL_OK ||
            HAL_FDCAN_Start(h) != HAL_OK)
            Error_Handler();
    }
}

/* ============================================================================
 * 核心接口：CAN 发送
 * ========================================================================== */
/* 帧属性只在公共入口验证一次；不补齐不可表达长度，避免隐式改变载荷。 */
int bsp_can_send(uint8_t bus, const bsp_can_frame_t *frame) {
    FDCAN_TxHeaderTypeDef h = {0};
    uint32_t dlc;
    if (bus >= BSP_CAN_BUS_NUM || frame == NULL || frame->extended > 1u || frame->fd > 1u ||
        frame->brs > 1u || (frame->brs && !frame->fd) ||
        (!frame->fd && frame->len > 8u) || (!s_bus[bus].fd && frame->fd) ||
        frame->id > (frame->extended ? 0x1fffffffu : 0x7ffu)) return -1;
    for (dlc = 0u; dlc < 16u && s_lengths[dlc] != frame->len; ++dlc) {}
    if (dlc == 16u) return -1;
    FDCAN_HandleTypeDef *handle = s_bus[bus].handle;
    if (HAL_FDCAN_GetTxFifoFreeLevel(handle) == 0u) {
        ++s_stats[bus].tx_drop;
        return 0;
    }
    h.Identifier = frame->id;
    h.IdType = frame->extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    h.TxFrameType = FDCAN_DATA_FRAME;
    h.DataLength = dlc;
    h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    h.BitRateSwitch = frame->brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    h.FDFormat = frame->fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    h.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    /* CubeMX 保持 FIFO 模式，按提交顺序入队，不按 CAN ID 重排。 */
    if (HAL_FDCAN_AddMessageToTxFifoQ(handle, &h, (uint8_t *)frame->data) != HAL_OK) {
        ++s_stats[bus].tx_error;
        return -1;
    }
    ++s_stats[bus].tx;
    return 1;
}

/* ============================================================================
 * HAL 回调：接收与错误处理
 * ========================================================================== */
/* 排空 FIFO0；缓存按最大 FD 帧分配，解析和状态机由接收者负责。 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *handle, uint32_t flags) {
    const uint8_t bus = bus_of(handle);
    FDCAN_RxHeaderTypeDef h;
    bsp_can_frame_t frame = {0};
    (void)flags;
    if (bus >= BSP_CAN_BUS_NUM) return;
    while (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &h, frame.data) == HAL_OK) {
        ++s_stats[bus].rx;
        if (h.RxFrameType != FDCAN_DATA_FRAME || h.DataLength >= 16u ||
            (h.IdType != FDCAN_STANDARD_ID && h.IdType != FDCAN_EXTENDED_ID) ||
            h.Identifier > (h.IdType == FDCAN_EXTENDED_ID ? 0x1fffffffu : 0x7ffu) ||
            (h.FDFormat != FDCAN_CLASSIC_CAN && h.FDFormat != FDCAN_FD_CAN) ||
            (h.FDFormat == FDCAN_CLASSIC_CAN && (h.DataLength > FDCAN_DLC_BYTES_8 || h.BitRateSwitch != FDCAN_BRS_OFF)) ||
            (!s_bus[bus].fd && h.FDFormat == FDCAN_FD_CAN) ||
            (h.BitRateSwitch != FDCAN_BRS_OFF && h.BitRateSwitch != FDCAN_BRS_ON)) {
            ++s_stats[bus].rx_reject;
            continue;
        }
        frame.id = h.Identifier;
        frame.len = s_lengths[h.DataLength];
        frame.extended = h.IdType == FDCAN_EXTENDED_ID;
        frame.fd = h.FDFormat == FDCAN_FD_CAN;
        frame.brs = h.BitRateSwitch == FDCAN_BRS_ON;
        if (s_rx[bus]) s_rx[bus](bus, &frame);
    }
}

/* bus-off 仅统计并清 INIT 请求恢复，不解释上层故障。 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *handle, uint32_t error) {
    const uint8_t bus = bus_of(handle);
    if (bus < BSP_CAN_BUS_NUM && (error & FDCAN_IT_BUS_OFF) != 0u) {
        ++s_stats[bus].bus_off;
        handle->Instance->CCCR &= ~FDCAN_CCCR_INIT;
    }
}

/* ============================================================================
 * 核心接口：硬件统计
 * ========================================================================== */
const bsp_can_stats_t *bsp_can_stats(uint8_t bus) {
    return bus < BSP_CAN_BUS_NUM ? &s_stats[bus] : NULL;
}
