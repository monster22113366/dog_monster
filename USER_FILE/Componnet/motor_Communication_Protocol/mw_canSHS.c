/* 
 * MW意为“Motor Wire”。即电机总线。这是守护兽的电调通信协议
 * RX 在中断中更新，控制任务通过 mwSHS_snapshot 获取完整快照。 */
#include "mw_canSHS.h"

/* ============================================================================
 * 节点状态缓存
 * ========================================================================== */
static mwSHS_ops_t s_ops;
/* 缺失任一临界区函数时整对停用，避免只上锁无法恢复。 */
static uint32_t platform_lock(void)
{ return s_ops.lock && s_ops.unlock ? s_ops.lock() : 0; }
static void platform_unlock(uint32_t key)
{ if (s_ops.lock && s_ops.unlock) s_ops.unlock(key); }


static mwSHS_motor_t table[MWSHS_BUS_NUM][MWSHS_NODE_NUM];
/* 0 未知，1 标准帧，2 扩展帧；仅心跳和编码器反馈提供学习依据。 */
static uint8_t frame_type[MWSHS_BUS_NUM][MWSHS_NODE_NUM];

/* ============================================================================
 * 核心接口：初始化
 * ========================================================================== */
/* 仅启动时调用，清空缓存和帧类型学习结果，保存发送回调。 */
void mwSHS_init(const mwSHS_ops_t *ops)
{ memset(table, 0, sizeof(table)); memset(frame_type, 0, sizeof(frame_type)); s_ops = ops ? *ops : (mwSHS_ops_t){0}; }

/* ============================================================================
 * 核心接口：状态读取与一致快照
 * ========================================================================== */
/* 原始缓存指针供串行诊断使用；控制任务使用下面的一致快照。 */
const mwSHS_motor_t *mwSHS_get(uint8_t bus, uint8_t node)
{ return bus < MWSHS_BUS_NUM && node < MWSHS_NODE_NUM ? &table[bus][node] : NULL; }


/* mwSHS_snapshot：在短临界区内复制整份节点状态，供控制任务读取。
 * 越界时清零输出；空输出指针不写入，均返回 0。 */
int mwSHS_snapshot(uint8_t bus, uint8_t node, mwSHS_motor_t *out) {
    if (out == NULL) return 0;
    if (bus >= MWSHS_BUS_NUM || node >= MWSHS_NODE_NUM) {
        memset(out, 0, sizeof(*out));
        return 0;
    }
    uint32_t key = platform_lock();
    memcpy(out, &table[bus][node], sizeof(*out));
    platform_unlock(key);
    return 1;
}

/* ============================================================================
 * 核心接口：发送
 * ========================================================================== */
/* 载荷不足八字节补零，超过八字节截断；未知帧类型保持双格式兼容。 */
void mwSHS_send(uint8_t bus, uint8_t node, uint8_t cmd, const void *payload, uint8_t len)
{
    mwSHS_frame_t frame = {0};
    if (!s_ops.tx || bus >= MWSHS_BUS_NUM || node >= MWSHS_NODE_NUM || cmd > 31) return;
    if (payload) memcpy(frame.data, payload, len > 8 ? 8 : len);
    frame.id = ((uint16_t)node << 5) | cmd;
    frame.len = 8u;
    const uint8_t type = frame_type[bus][node];
    frame.extended = type == 2u;
    /* 未知时先标准再扩展；首次 FIFO 已满则停止，与原硬件路径一致。
     * 首次 平台硬件 入队失败仍尝试扩展帧，不把硬件错误解释为电机故障。 */
    const int sent = s_ops.tx(bus, &frame);
    if (type == 0u && sent != 0) {
        frame.extended = 1u;
        (void)s_ops.tx(bus, &frame);
    }
}

/* ============================================================================
 * 内部辅助函数：字段解码
 * ========================================================================== */
/* 用 memcpy 读取两个四字节字段，避免未对齐访问。 */
static void read_pair(void *a, void *b, const uint8_t *data)
{ memcpy(a, data, 4); memcpy(b, data + 4, 4); }

/* ============================================================================
 * 核心接口：接收与帧类型学习
 * ========================================================================== */
/* 硬件帧属性由 BSP 保证；这里检查 MW 地址、载荷长度及命令。
 * 仅心跳和编码器响应学习标准/扩展格式，其余已支持响应只更新缓存。 */
void mwSHS_recv(uint8_t bus, const mwSHS_frame_t *frame)
{
    if (frame == NULL || frame->len != 8u || frame->id > 0x7ffu || bus >= MWSHS_BUS_NUM) return;
    const uint16_t can_id = (uint16_t)frame->id;
    const uint16_t node = can_id >> 5;
    const uint8_t *data = frame->data;
    mwSHS_motor_t *m;
    if (node >= MWSHS_NODE_NUM) return;
    if ((can_id & 31u) == MWSHS_CMD_HEARTBEAT || (can_id & 31u) == MWSHS_CMD_ENCODER)
        frame_type[bus][node] = frame->extended ? 2u : 1u;
    m = &table[bus][node];
    switch (can_id & 31) {
    /* 心跳：B0~B3 轴错误、B4 状态、B5 错误/轨迹完成位、B7 心跳计数。 */
    case MWSHS_CMD_HEARTBEAT:
        memcpy(&m->axis_error, data, 4); m->state = data[4];
        m->motor_err = data[5] & 1; m->encoder_err = (data[5] >> 1) & 1;
        m->controller_err = (data[5] >> 2) & 1; m->system_err = (data[5] >> 3) & 1;
        m->traj_done = data[5] >> 7; m->life = data[7];
        break;
    /* 错误：B0~B7 整体存为 u64；u32 类型高 4 字节为 0；类型由调用方记录。 */
    case MWSHS_CMD_ERROR: memcpy(&m->error, data, 8); break;
    /* 编码器：转子侧位置 turns、速度 turns/s。 */
    case MWSHS_CMD_ENCODER: read_pair(&m->pos, &m->vel, data); break;
    /* 编码器计数：多圈、单圈，均为 i32 counts。 */
    case MWSHS_CMD_COUNT: read_pair(&m->shadow_count, &m->count_in_cpr, data); break;
    /* 电流：设定值、实测值，单位 A。 */
    case MWSHS_CMD_IQ: read_pair(&m->iq_set, &m->iq_meas, data); break;
    /* 母线：电压 V、电流 A。 */
    case MWSHS_CMD_BUS: read_pair(&m->bus_v, &m->bus_i, data); break;
    /* 力矩：设定值、实测值，单位 Nm。 */
    case MWSHS_CMD_TORQUES: read_pair(&m->torque_set, &m->torque, data); break;
    /* 功率：电功率、机械功率，单位 W。 */
    case MWSHS_CMD_POWERS: read_pair(&m->elec_power, &m->mech_power, data); break;
    /* 未实现解析的命令直接忽略，不累计接收计数。 */
    default: return;
    }
    ++m->rx_count;
}
