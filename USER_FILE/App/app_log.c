/*
 * 本文件属于 App 应用层，实现通用调试记录器。
 * 数据布局、帧格式与调试接口见 app_log.h；记录内容由各模块导出的 g_* 描述符表决定。
 * 仅依赖 app_log.h、模块描述符和 app_ctrl 控制时基，不访问硬件寄存器。
 */
#include "app_log.h"
#include "app_imu.h"
#include "app_rc.h"
#include "app_legs.h"
#include "app_chassis.h"
#include "app_motorSHS.h"
#include "app_ctrl.h"
#include <string.h>

#ifndef APP_LOG_MAGIC
#define APP_LOG_MAGIC 0x4C4F4F50u
#endif
#ifndef APP_LOG_VERSION
#define APP_LOG_VERSION 1u
#endif
#ifndef APP_LOG_FRAME_MS
/* 每帧周期 ms：控制链 2 ms，取 12 ms（6 拍）便于整除且降低带宽；只影响时间分辨率，不改变数据来源。 */
#define APP_LOG_FRAME_MS 12u
#endif

_Static_assert(APP_LOG_FRAME_MS >= APP_CTRL_PERIOD_MS &&
               APP_LOG_FRAME_MS % APP_CTRL_PERIOD_MS == 0u && APP_LOG_FRAME_MS <= UINT16_MAX,
               "log period must be a positive multiple of control period and fit u16");
_Static_assert(APP_LOG_CAPACITY > sizeof(app_log_hdr_t), "log buffer must hold its header");

volatile app_log_debug_t g_log;

static const sim_vars_t *const s_groups[] = {
    &g_imu_vars, 
    &g_rc_vars,
    &g_legs_vars,
    &g_chassis_vars,
    &g_motorSHS_vars, 
    &g_ctrl_vars,
};
#define LOG_GROUP_NUM (sizeof(s_groups) / sizeof(s_groups[0]))

static uint32_t s_frame_bytes;
static uint32_t s_frame_max;
static uint32_t s_tick_count;

/* var_bytes：按描述符类型与元素个数计算原始载荷长度，不包含结构体填充。 */
static uint32_t var_bytes(const sim_var_desc_t *v) {
    const uint32_t n = v->extra == 0u ? 1u : v->extra;
    switch (v->type) {
    case SIM_VAR_U8: case SIM_VAR_I8: return n;
    case SIM_VAR_U16: case SIM_VAR_I16: return n * 2u;
    default: return n * 4u;
    }
}

/* frame_bytes_of：帧头、全部模块载荷与校验组成固定帧长。 */
static uint32_t frame_bytes_of(void) {
    uint32_t total = APP_LOG_FRAME_OVERHEAD;
    for (uint32_t g = 0u; g < LOG_GROUP_NUM; g++) {
        for (uint16_t i = 0u; i < s_groups[g]->n; i++)
            total += var_bytes(&s_groups[g]->vars[i]);
    }
    return total;
}

/* app_log_init：根据模块描述符计算布局，仅复位普通 RAM 中的记录器状态。 */
void app_log_init(void) {
    memset((void *)&g_log, 0, sizeof(g_log));
    s_frame_bytes = frame_bytes_of();
    s_frame_max = (APP_LOG_CAPACITY - sizeof(app_log_hdr_t)) / s_frame_bytes;
    s_tick_count = 0u;
}

/* app_log_start：初始化固定 32 字节缓冲区头并开始新一轮记录。 */
void app_log_start(void) {
    app_log_hdr_t *const h = (app_log_hdr_t *)APP_LOG_BASE;
    memset(h, 0, sizeof(*h));
    h->magic = APP_LOG_MAGIC;
    h->version = APP_LOG_VERSION;
    h->frame_ms = APP_LOG_FRAME_MS;
    h->capacity = APP_LOG_CAPACITY;
    h->frame_bytes = s_frame_bytes;
    h->frame_max = s_frame_max;
    s_tick_count = 0u;
    h->count = 0u;
    g_log.count = 0u;
    g_log.seconds = 0.0f;
    g_log.state = 1u;
    g_log.enabled = 1u;
}

/* app_log_tick：处理启动请求，每六拍按描述符搬运载荷并附加逐字节和校验。 */
void app_log_tick(void) {
    if (g_log.start != 0u) {
        g_log.start = 0u;
        app_log_start();
        return;
    }
    if (g_log.enabled == 0u)
        return;
    s_tick_count++;
    if (s_tick_count < (uint32_t)(APP_LOG_FRAME_MS / APP_CTRL_PERIOD_MS))
        return;
    s_tick_count = 0u;

    app_log_hdr_t *const h = (app_log_hdr_t *)APP_LOG_BASE;
    const uint32_t index = h->count;
    if (index >= s_frame_max) {
        g_log.state = 2u;
        g_log.enabled = 0u;
        return;
    }
    uint8_t *const frame = (uint8_t *)APP_LOG_BASE + sizeof(app_log_hdr_t) + index * s_frame_bytes;
    const uint32_t t_ms = app_ctrl_ms();
    const uint16_t flags = (uint16_t)((g_imu.ok != 0u ? 1u : 0u) | (g_legs.fault != 0u ? 2u : 0u) |
                                      (g_motor_all_running() != 0u ? 4u : 0u));
    frame[0] = (uint8_t)(index & 0xffu);
    frame[1] = (uint8_t)((index >> 8) & 0xffu);
    /* 时间戳用完整 32 位：12 ms/帧、最多 2 万帧，16 位会回绕多次。 */
    const uint32_t ts = (uint32_t)t_ms;
    frame[2] = (uint8_t)(ts & 0xffu);
    frame[3] = (uint8_t)((ts >> 8) & 0xffu);
    frame[4] = (uint8_t)((ts >> 16) & 0xffu);
    frame[5] = (uint8_t)((ts >> 24) & 0xffu);
    frame[6] = (uint8_t)(flags & 0xffu);
    frame[7] = (uint8_t)((flags >> 8) & 0xffu);
    /* 顺序拼接避免固定二维偏移表对模块数和描述符数施加隐含上限。 */
    uint32_t off = 8u;
    for (uint32_t g = 0u; g < LOG_GROUP_NUM; g++) {
        for (uint16_t i = 0u; i < s_groups[g]->n; i++) {
            const sim_var_desc_t *const v = &s_groups[g]->vars[i];
            const uint32_t bytes = var_bytes(v);
            memcpy(frame + off, v->addr, bytes);
            off += bytes;
        }
    }
    /* 按字节异或：帧长为奇数也不会与校验字段重叠；单字节变化必被检出。 */
    uint16_t sum = 0u;
    for (uint32_t b = 0u; b < s_frame_bytes - 2u; b++)
        sum = (uint16_t)(sum ^ frame[b]);
    frame[s_frame_bytes - 2u] = (uint8_t)(sum & 0xffu);
    frame[s_frame_bytes - 1u] = (uint8_t)((sum >> 8) & 0xffu);

    h->count = index + 1u;
    g_log.count = index + 1u;
    g_log.seconds = (float)(index + 1u) * (float)APP_LOG_FRAME_MS * 0.001f;
    if (index + 1u == s_frame_max) {
        g_log.state = 2u;
        g_log.enabled = 0u;
    }
}

/* app_log_frame_ms：返回记录帧周期，单位为控制节拍 ms。 */
uint16_t app_log_frame_ms(void) {
    return APP_LOG_FRAME_MS;
}
