/* 三模式基础控制；参数事务只写 Control Mode，不保存 Flash。 */
#include "app_motorQZB.h"
#include "bsp_critical.h"
#include "bsp_time.h"
#include <math.h>
#include <string.h>
#include "bsp_can.h"

static int qzb_tx(uint8_t bus, const mwQZB_frame_t *f)
{
    if (!f) return -1;
    bsp_can_frame_t bf = {0};
    bf.id = f->id; bf.len = f->len;
    memcpy(bf.data, f->data, sizeof(bf.data));
    bf.extended = f->extended; bf.fd = f->fd; bf.brs = f->brs;
    return bsp_can_send(bus, &bf);
}

static void qzb_rx(uint8_t bus, const bsp_can_frame_t *bf)
{
    if (!bf) return;
    mwQZB_frame_t f = {0};
    f.id = bf->id; f.len = bf->len;
    memcpy(f.data, bf->data, sizeof(f.data));
    f.extended = bf->extended; f.fd = bf->fd; f.brs = bf->brs;
    mwQZB_recv(bus, &f);
}

static uint32_t qzb_lock(void) { return bsp_critical_enter(); }
static void qzb_unlock(uint32_t key) { bsp_critical_leave(key); }
static uint32_t qzb_millis(void) { return bsp_millis(); }
_Static_assert(MWQZB_BUS_NUM == BSP_CAN_BUS_NUM, "CAN bus counts must agree");
static const mwQZB_ops_t s_can_ops = { .tx = qzb_tx, .millis = qzb_millis, .lock = qzb_lock, .unlock = qzb_unlock };


extern volatile uint8_t g_motorSHS_kill;
volatile app_motorQZB_t g_motorQZB[APP_MOTORQZB_COUNT];
volatile uint8_t g_motorQZB_kill;
static volatile uint8_t s_kill_pending;
static const struct { uint8_t bus, id; } s_inst[APP_MOTORQZB_COUNT] = {
    {BSP_CAN_BUS3, 1}, {BSP_CAN_BUS3, 2}, /* LF、RF */
    {BSP_CAN_BUS3, 3}, {BSP_CAN_BUS3, 4}  /* LB、RB */
};
static struct {
    uint8_t armed, stopping, requested, busy, stop_pending;
    uint32_t since, last_stop, last_retry, param_count, busy_since;
} s_ctx[APP_MOTORQZB_COUNT];

static float clip(float v, float lo, float hi)
{ return v < lo ? lo : (v > hi ? hi : v); }

/* 三态发送结果：成功清忙；FIFO 满逐拍重试；真错误或连续忙超时立即报错。 */
static int tx_result(unsigned i, app_motorQZB_t *g, uint32_t now, int result)
{
    if (result == 1) { s_ctx[i].busy = 0; return 1; }
    if (result < 0) return -1;
    if (!s_ctx[i].busy) {
        s_ctx[i].busy = 1; s_ctx[i].busy_since = now;
    }
    if ((uint32_t)(now - s_ctx[i].busy_since) >= APP_MOTORQZB_TX_BUSY_MS) return -1;
    ++g->tx_busy;
    return 0;
}

static int send_state(unsigned i, app_motorQZB_t *g, uint32_t now, mwQZB_state_cmd_t cmd)
{
    uint8_t d[8];
    mwQZB_pack_state(d, cmd);
    return tx_result(i, g, now, mwQZB_send(s_inst[i].bus, MWQZB_STATE_ID_BASE, s_inst[i].id, d));
}

static int stop(unsigned i, app_motorQZB_t *g, uint32_t now, uint8_t force)
{
    if (!force && !s_ctx[i].stop_pending &&
        (uint32_t)(now - s_ctx[i].last_stop) < APP_MOTORQZB_RETRY_MS) return 1;
    int result = send_state(i, g, now, MWQZB_STATE_DISABLE);
    s_ctx[i].stop_pending = result == 0;
    if (result == 1) s_ctx[i].last_stop = now;
    return result;
}

static void fault(unsigned i, app_motorQZB_t *g, app_motorQZB_fault_t code, uint32_t now)
{
    uint8_t changed = g->phase != APP_QZB_FAULT || g->fault != code;
    g->phase = APP_QZB_FAULT; g->fault = code;
    s_ctx[i].armed = 0; s_ctx[i].stopping = 0;
    (void)stop(i, g, now, changed);
}

static int control(unsigned i, app_motorQZB_t *g, uint32_t now, uint8_t safe)
{
    uint8_t d[8];
    if (safe)
        mwQZB_pack_mit(d, g->fb.pos, 0, 0, 0, 0);
    else if (g->mode == MOTORQZB_MODE_TORQUE)
        mwQZB_pack_mit(d, 0, 0, 0, 0, clip(g->tq, -APP_MOTORQZB_TQ_LIMIT, APP_MOTORQZB_TQ_LIMIT));
    else if (g->mode == MOTORQZB_MODE_SPEED)
        /* kp=0，位置字段固定为零，不依赖累计转动后饱和的反馈位置。 */
        mwQZB_pack_mit(d, 0, clip(g->vel, -APP_MOTORQZB_VEL_LIMIT, APP_MOTORQZB_VEL_LIMIT),
                    0, clip(g->vel_kd, 0, APP_MOTORQZB_VEL_KD_LIMIT), 0);
    else
        mwQZB_pack_mit(d, clip(g->pos, MWQZB_POS_MIN, MWQZB_POS_MAX), 0,
                    clip(g->pos_kp, 0, APP_MOTORQZB_POS_KP_LIMIT),
                    clip(g->pos_kd, 0, APP_MOTORQZB_POS_KD_LIMIT), 0);
    return tx_result(i, g, now, mwQZB_send(s_inst[i].bus, MWQZB_CONTROL_ID_BASE, s_inst[i].id, d));
}

static void begin_enable(unsigned i, app_motorQZB_t *g, uint32_t now)
{
    /* FIFO 顺序：先覆盖旧目标，再使能；安全帧忙时保留原阶段以便重入。 */
    int result = control(i, g, now, 1);
    if (result < 0) fault(i, g, APP_QZB_FAULT_TX, now);
    if (result != 1) return;
    g->phase = APP_QZB_ENABLING;
    s_ctx[i].since = now;
    result = send_state(i, g, now, MWQZB_STATE_ENABLE);
    s_ctx[i].last_retry = result == 1 ? now : now - APP_MOTORQZB_RETRY_MS;
    if (result < 0) fault(i, g, APP_QZB_FAULT_TX, now);
}

static void step(unsigned i, app_motorQZB_t *g, uint8_t killed)
{
    int result;
    mwQZB_param_t param;
    mwQZB_get_feedback(s_inst[i].bus, s_inst[i].id, &g->fb);
    mwQZB_get_param(s_inst[i].bus, s_inst[i].id, &param);
    uint32_t now = bsp_millis(); /* 先拷贝反馈再读时钟，避免时间差下溢。 */
    const mwQZB_mode_t target = MWQZB_MODE_MIT; /* 三种应用模式共用电机 MIT 模式。 */
    g->rx_count = g->fb.count;
    g->online = g->fb.count && (uint32_t)(now - g->fb.time) < APP_MOTORQZB_RX_TIMEOUT_MS;
    if (killed) { fault(i, g, APP_QZB_FAULT_KILLED, now); return; }
    if (!g->online) {
        (void)stop(i, g, now, g->phase != APP_QZB_OFFLINE);
        g->phase = APP_QZB_OFFLINE; g->fault = APP_QZB_FAULT_OFFLINE; g->active_mode = 0;
        s_ctx[i].armed = 0; s_ctx[i].stopping = 0;
        return;
    }
    if (g->clear_error && !g->enable && !g->fb.enabled) {
        result = send_state(i, g, now, MWQZB_STATE_CLEAR);
        if (result < 0) fault(i, g, APP_QZB_FAULT_TX, now);
        if (result != 1) return;
        g_motorQZB[i].clear_error = 0;
    }
    if (g->fb.error & MWQZB_FAULT_MASK) { fault(i, g, APP_QZB_FAULT_MOTOR, now); return; }
    if (g->phase == APP_QZB_FAULT) {
        (void)stop(i, g, now, 0);
        if (!g->enable && !g->fb.enabled) {
            g->phase = APP_QZB_IDLE; g->fault = APP_QZB_FAULT_NONE; s_ctx[i].armed = 1;
        }
        return;
    }
    if (g->phase == APP_QZB_OFFLINE) {
        g->phase = APP_QZB_IDLE; g->fault = APP_QZB_FAULT_NONE;
    }
    if (!g->enable) s_ctx[i].armed = 1;
    if (g->enable && s_ctx[i].armed) {
        if (g->mode < MOTORQZB_MODE_TORQUE || g->mode > MOTORQZB_MODE_POSITION) {
            fault(i, g, APP_QZB_FAULT_BAD_MODE, now); return;
        }
        if (!isfinite(g->tq) || !isfinite(g->vel) || !isfinite(g->pos) ||
            !isfinite(g->pos_kp) || !isfinite(g->pos_kd) ||
            !isfinite(g->vel_kd)) {
            fault(i, g, APP_QZB_FAULT_BAD_COMMAND, now); return;
        }
    }
    /* 停机等待留在原阶段；收到失能反馈后才回 IDLE，不增加第七态。 */
    if (!s_ctx[i].stopping && g->phase != APP_QZB_IDLE &&
        (!g->enable || (g->active_mode != 0 && g->active_mode != target) ||
         (g->phase == APP_QZB_CONFIG && g->fb.enabled))) {
        s_ctx[i].stopping = 1; s_ctx[i].since = now;
        if (stop(i, g, now, 1) < 0) fault(i, g, APP_QZB_FAULT_TX, now);
        return;
    }
    if (s_ctx[i].stopping) {
        result = stop(i, g, now, 0);
        if (result < 0) fault(i, g, APP_QZB_FAULT_TX, now);
        if (result != 1) return;
        if (!g->fb.enabled) { s_ctx[i].stopping = 0; g->phase = APP_QZB_IDLE; }
        else if ((uint32_t)(now - s_ctx[i].since) >= APP_MOTORQZB_STEP_TIMEOUT_MS)
            fault(i, g, APP_QZB_FAULT_DISABLE_TIMEOUT, now);
        return;
    }
    switch (g->phase) {
    case APP_QZB_IDLE:
        /* 已确认 MIT 且本拍启动时，安全帧+使能帧替代无意义的失能保活。 */
        if (g->enable && s_ctx[i].armed && !g->fb.enabled && g->active_mode == target) {
            s_ctx[i].requested = g->mode;
            begin_enable(i, g, now);
            break;
        }
        result = stop(i, g, now, 0);
        if (result < 0) fault(i, g, APP_QZB_FAULT_TX, now);
        if (result != 1) break;
        if (!g->enable || !s_ctx[i].armed || g->fb.enabled) break;
        s_ctx[i].requested = g->mode;
        g->phase = APP_QZB_CONFIG;
        g->active_mode = 0; /* 写参数后旧模式不再可信，即使本次回包丢失。 */
        s_ctx[i].since = now; s_ctx[i].param_count = param.count;
        s_ctx[i].last_retry = now - APP_MOTORQZB_RETRY_MS;
        /* fall through */
    case APP_QZB_CONFIG:
        result = stop(i, g, now, 0);
        if (result < 0) { fault(i, g, APP_QZB_FAULT_TX, now); break; }
        if ((uint32_t)(now - s_ctx[i].since) >= APP_MOTORQZB_STEP_TIMEOUT_MS) {
            fault(i, g, APP_QZB_FAULT_PARAM_TIMEOUT, now); break;
        }
        if (result == 0) break;
        if (param.count != s_ctx[i].param_count && param.index == MWQZB_PARAM_CONTROL_MODE) {
            if (param.raw != (uint32_t)target) { fault(i, g, APP_QZB_FAULT_MODE_MISMATCH, now); break; }
            g->active_mode = target;
            begin_enable(i, g, now);
            break;
        }
        if ((uint32_t)(now - s_ctx[i].last_retry) >= APP_MOTORQZB_RETRY_MS) {
            uint8_t d[8];
            mwQZB_pack_param(d, MWQZB_PARAM_CONTROL_MODE, target);
            result = tx_result(i, g, now, mwQZB_send(s_inst[i].bus, MWQZB_PARAM_ID_BASE, s_inst[i].id, d));
            if (result == 1) s_ctx[i].last_retry = now;
            else if (result < 0) fault(i, g, APP_QZB_FAULT_TX, now);
        }
        break;
    case APP_QZB_ENABLING:
        if (!g->fb.enabled && (uint32_t)(now - s_ctx[i].since) >= APP_MOTORQZB_STEP_TIMEOUT_MS) {
            fault(i, g, APP_QZB_FAULT_ENABLE_TIMEOUT, now); break;
        }
        result = control(i, g, now, 1);
        if (result < 0) fault(i, g, APP_QZB_FAULT_TX, now);
        if (result != 1) break;
        if (g->fb.enabled) {
            g->phase = APP_QZB_RUN; g->fault = APP_QZB_FAULT_NONE;
            if (g->mode == MOTORQZB_MODE_POSITION) g_motorQZB[i].pos = g->pos = g->fb.pos;
            s_ctx[i].requested = g->mode;
        } else if ((uint32_t)(now - s_ctx[i].last_retry) >= APP_MOTORQZB_RETRY_MS) {
            result = send_state(i, g, now, MWQZB_STATE_ENABLE);
            if (result == 1) s_ctx[i].last_retry = now;
            else if (result < 0) fault(i, g, APP_QZB_FAULT_TX, now);
        }
        break;
    case APP_QZB_RUN:
        /* 应用模式互切不失能、不重配；同时更新快照，保证当拍使用新目标。 */
        if (g->mode != s_ctx[i].requested) {
            /* 已知限制：累计位置反馈在 ±12.5 rad 饱和，对齐值也受此限制。 */
            if (g->mode == MOTORQZB_MODE_POSITION) g_motorQZB[i].pos = g->pos = g->fb.pos;
            s_ctx[i].requested = g->mode;
        }
        if (!g->fb.enabled) fault(i, g, APP_QZB_FAULT_LOST_ENABLE, now);
        else if (control(i, g, now, 0) < 0) fault(i, g, APP_QZB_FAULT_TX, now);
        break;
    default: break;
    }
}

int app_motorQZB_init(void)
{
    memset((void *)g_motorQZB, 0, sizeof(g_motorQZB));
    memset(s_ctx, 0, sizeof(s_ctx));
    g_motorQZB_kill = s_kill_pending = 0;
    mwQZB_init(&s_can_ops);
    mwQZB_bus_register(BSP_CAN_BUS3);
    bsp_can_set_rx_callback(BSP_CAN_BUS3, qzb_rx);
    for (unsigned i = 0; i < APP_MOTORQZB_COUNT; ++i) {
        g_motorQZB[i].mode = MOTORQZB_MODE_TORQUE;
        g_motorQZB[i].pos_kp = APP_MOTORQZB_POS_KP_DEFAULT;
        g_motorQZB[i].pos_kd = APP_MOTORQZB_POS_KD_DEFAULT;
        g_motorQZB[i].vel_kd = APP_MOTORQZB_VEL_KD_DEFAULT;
        s_ctx[i].last_stop = bsp_millis() - APP_MOTORQZB_RETRY_MS;
    }
    app_motorQZB_request_clear();
    return 1;
}

void app_motorQZB_request_clear(void)
{
    uint32_t key = bsp_critical_enter();
    for (unsigned i = 0; i < APP_MOTORQZB_COUNT; ++i)
        g_motorQZB[i].clear_error = 1u;
    bsp_critical_leave(key);
}

void app_motorQZB_tick(void)
{
    app_motorQZB_t input[APP_MOTORQZB_COUNT];
    uint32_t key = bsp_critical_enter();
    uint8_t killed = g_motorSHS_kill || g_motorQZB_kill || s_kill_pending;
    s_kill_pending = 0;
    for (unsigned i = 0; i < APP_MOTORQZB_COUNT; ++i) input[i] = g_motorQZB[i];
    bsp_critical_leave(key);
    for (unsigned i = 0; i < APP_MOTORQZB_COUNT; ++i) {
        step(i, &input[i], killed);
        key = bsp_critical_enter();
        g_motorQZB[i].phase = input[i].phase; g_motorQZB[i].fault = input[i].fault;
        g_motorQZB[i].active_mode = input[i].active_mode; g_motorQZB[i].fb = input[i].fb;
        g_motorQZB[i].tx_busy = input[i].tx_busy;
        g_motorQZB[i].online = input[i].online; g_motorQZB[i].rx_count = input[i].rx_count;
        bsp_critical_leave(key);
    }
}

void app_motorQZB_kill(void)
{
    uint32_t key = bsp_critical_enter();
    g_motorQZB_kill = s_kill_pending = 1;
    bsp_critical_leave(key);
}

void app_motorQZB_kill_release(void)
{
    g_motorQZB_kill = 0;
}
