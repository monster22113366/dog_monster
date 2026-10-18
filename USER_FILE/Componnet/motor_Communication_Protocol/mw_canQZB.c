/* 
 * MW意为“Motor Wire”。即电机总线。这是泉智博的电调通信协议
 * RX 在中断中更新，控制任务通过 mwSHS_snapshot 获取完整快照。 */
#include "mw_canQZB.h"
#include <string.h>

static mwQZB_ops_t s_ops;
/* 缺失任一临界区函数时整对停用，避免只上锁无法恢复。 */
static uint32_t platform_lock(void)
{ return s_ops.lock && s_ops.unlock ? s_ops.lock() : 0; }
static void platform_unlock(uint32_t key)
{ if (s_ops.lock && s_ops.unlock) s_ops.unlock(key); }

static mwQZB_feedback_t s_fb[MWQZB_BUS_NUM][MWQZB_MAX_ID + 1u];
static mwQZB_param_t s_param[MWQZB_BUS_NUM][MWQZB_MAX_ID + 1u];
static uint8_t s_registered[MWQZB_BUS_NUM];

static uint16_t quantize(float v, float min, float max, uint16_t scale)
{
    if (v <= min) return 0;
    if (v >= max) return scale;
    return (uint16_t)((v - min) / (max - min) * scale);
}

static float decode(uint16_t raw, float min, float max, uint16_t scale)
{
    if (raw == 0) return min;
    if (raw == scale) return max;
    return (float)raw / scale * (max - min) + min;
}

void mwQZB_pack_mit(uint8_t d[8], float pos, float vel, float kp, float kd, float tq)
{
    uint16_t p = quantize(pos, MWQZB_POS_MIN, MWQZB_POS_MAX, 65535u);
    uint16_t v = quantize(vel, MWQZB_VEL_MIN, MWQZB_VEL_MAX, 4095u);
    uint16_t k = quantize(kp, MWQZB_KP_MIN, MWQZB_KP_MAX, 4095u);
    uint16_t b = quantize(kd, MWQZB_KD_MIN, MWQZB_KD_MAX, 4095u);
    uint16_t t = quantize(tq, MWQZB_TQ_MIN, MWQZB_TQ_MAX, 4095u);
    d[0] = (uint8_t)(p >> 8); d[1] = (uint8_t)p;
    d[2] = (uint8_t)(v >> 4); d[3] = (uint8_t)((v << 4) | (k >> 8));
    d[4] = (uint8_t)k; d[5] = (uint8_t)(b >> 4);
    d[6] = (uint8_t)((b << 4) | (t >> 8)); d[7] = (uint8_t)t;
}

void mwQZB_pack_vel(uint8_t d[8], float vel, float kp, float kd, float ki)
{
    uint16_t v = quantize(vel, MWQZB_VEL_MIN, MWQZB_VEL_MAX, 65535u);
    uint16_t k = quantize(kp, MWQZB_KP_MIN, MWQZB_KP_MAX, 4095u);
    uint16_t b = quantize(kd, MWQZB_KD_MIN, MWQZB_KD_MAX, 4095u);
    uint16_t i = quantize(ki, MWQZB_KI_MIN, MWQZB_KI_MAX, 4095u);
    d[0] = (uint8_t)(v >> 8); d[1] = (uint8_t)v;
    /* 单播表 Kp 低位文字误写 Byte4，按位栏及广播表使用 Byte3[7:4]。 */
    d[2] = (uint8_t)(k >> 4); d[3] = (uint8_t)((k << 4) | (b >> 8));
    d[4] = (uint8_t)b; d[5] = (uint8_t)(i >> 4);
    d[6] = (uint8_t)(i << 4); d[7] = 0xACu;
}

void mwQZB_pack_state(uint8_t d[8], mwQZB_state_cmd_t cmd)
{
    memset(d, 0xFF, 8);
    d[7] = (uint8_t)cmd;
}

void mwQZB_pack_param(uint8_t d[8], uint8_t index, uint32_t raw)
{
    d[0] = 0x67; d[1] = index;
    for (unsigned i = 0; i < 4; ++i) d[2 + i] = (uint8_t)(raw >> (8 * i));
    d[6] = 0x15; d[7] = 0x76;
}

int mwQZB_send(uint8_t bus, uint16_t ident_base, uint8_t id, const uint8_t data[8])
{
    mwQZB_frame_t f = {.id = ident_base + id, .len = 8, .fd = MWQZB_USE_FD, .brs = (MWQZB_USE_FD && MWQZB_USE_BRS)};
    memcpy(f.data, data, 8);
    return s_ops.tx ? s_ops.tx(bus, &f) : -1;
}

void mwQZB_init(const mwQZB_ops_t *ops)
{
    s_ops = ops ? *ops : (mwQZB_ops_t){0};
    memset(s_fb, 0, sizeof(s_fb));
    memset(s_param, 0, sizeof(s_param));
    memset(s_registered, 0, sizeof(s_registered));
}

void mwQZB_bus_register(uint8_t bus)
{
    if (bus >= MWQZB_BUS_NUM) return;
    memset(s_fb[bus], 0, sizeof(s_fb[bus]));
    memset(s_param[bus], 0, sizeof(s_param[bus]));
    s_registered[bus] = 1;
}

void mwQZB_recv(uint8_t bus, const mwQZB_frame_t *f)
{
    if (bus >= MWQZB_BUS_NUM || !s_registered[bus] || !f || f->len != 8 || f->extended) return;
    const uint8_t *d = f->data;
    if (f->id >= 1 && f->id <= MWQZB_MAX_ID) {
        mwQZB_feedback_t *fb = &s_fb[bus][f->id];
        fb->pos = decode((uint16_t)((d[0] << 8) | d[1]), MWQZB_POS_MIN, MWQZB_POS_MAX, 65535u);
        fb->vel = decode((uint16_t)((d[2] << 4) | (d[3] >> 4)), MWQZB_VEL_MIN, MWQZB_VEL_MAX, 4095u);
        fb->torque = decode((uint16_t)(((d[3] & 15u) << 8) | d[4]), MWQZB_TQ_MIN, MWQZB_TQ_MAX, 4095u);
        fb->temperature = (int16_t)(d[5] - 40);
        fb->status = (uint16_t)((d[6] << 8) | d[7]);
        fb->enabled = (uint8_t)(fb->status & 1u);
        fb->error = fb->status & MWQZB_FAULT_MASK;
        fb->time = (s_ops.millis ? s_ops.millis() : 0);
        ++fb->count;
    } else if (f->id > MWQZB_PARAM_ID_BASE && f->id <= MWQZB_PARAM_ID_BASE + MWQZB_MAX_ID &&
               d[0] == f->id - MWQZB_PARAM_ID_BASE && d[6] == 0 && d[7] == 0xFF) {
        mwQZB_param_t *p = &s_param[bus][d[0]];
        p->index = d[1];
        p->raw = (uint32_t)d[2] | ((uint32_t)d[3] << 8) | ((uint32_t)d[4] << 16) | ((uint32_t)d[5] << 24);
        ++p->count;
    }
}

int mwQZB_get_feedback(uint8_t bus, uint8_t id, mwQZB_feedback_t *out)
{
    if (bus >= MWQZB_BUS_NUM || !s_registered[bus] || !id || id > MWQZB_MAX_ID || !out) return 0;
    uint32_t key = platform_lock();
    *out = s_fb[bus][id];
    platform_unlock(key);
    return 1;
}

int mwQZB_get_param(uint8_t bus, uint8_t id, mwQZB_param_t *out)
{
    if (bus >= MWQZB_BUS_NUM || !s_registered[bus] || !id || id > MWQZB_MAX_ID || !out) return 0;
    uint32_t key = platform_lock();
    *out = s_param[bus][id];
    platform_unlock(key);
    return 1;
}
