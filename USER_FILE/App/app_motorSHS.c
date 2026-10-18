/*
 * 本文件属于 App 应用层，借助 mw_canSHS 组织电调协议命令，并通过 MW Component 获取一致反馈。
 * 对外实现电机初始化和周期状态机；由系统初始化代码及 app_ctrl 定时中断调用。
 * 依赖 app_motorSHS.h、mw_canSHS 协议接口和 mwSHS_snapshot 一致快照接口。
 * 调试方法：在调试器 Live Watch 中修改 g_motorSHS[i].enable、g_motorSHS[i].mode、g_motorSHS[i].target；g_motorSHS[i].mode 取 1~3。
 * 出故障时在 Live Watch 写 g_motorSHS_kill=1 一键失能。
 * 电调启用轴级看门狗时，本文件在进入闭环前与 WAIT_CL/SWITCH 期间用零力矩帧喂狗；RUN 期间由运动指令喂狗。
 */
#include "app_motorSHS.h"
#include "bsp_time.h"
#include <math.h>
#include <string.h>
#include "bsp_can.h"
#include "bsp_critical.h"

static int shs_tx(uint8_t bus, const mwSHS_frame_t *f)
{
    if (!f) return -1;
    bsp_can_frame_t bf = {0};
    bf.id = f->id; bf.len = f->len;
    memcpy(bf.data, f->data, sizeof(bf.data));
    bf.extended = f->extended; bf.fd = f->fd; bf.brs = f->brs;
    return bsp_can_send(bus, &bf);
}

static void shs_rx(uint8_t bus, const bsp_can_frame_t *bf)
{
    if (!bf) return;
    mwSHS_frame_t f = {0};
    f.id = bf->id; f.len = bf->len;
    memcpy(f.data, bf->data, sizeof(f.data));
    f.extended = bf->extended; f.fd = bf->fd; f.brs = bf->brs;
    mwSHS_recv(bus, &f);
}

static uint32_t shs_lock(void) { return bsp_critical_enter(); }
static void shs_unlock(uint32_t key) { bsp_critical_leave(key); }
static const mwSHS_ops_t s_can_ops = { .tx = shs_tx, .lock = shs_lock, .unlock = shs_unlock };


#ifndef APP_RX_TIMEOUT_MS
/* 多久没有收到新 CAN 帧就判为离线，单位毫秒。 */
#define APP_RX_TIMEOUT_MS 200u
#endif
#ifndef APP_CL_TIMEOUT_MS
/* 等待电调确认闭环或切换空闲的最长时间，单位毫秒。 */
#define APP_CL_TIMEOUT_MS 500u
#endif
#ifndef APP_FB_PERIOD_MS
/* 电流和母线电压轮询的发送间隔，单位毫秒。 */
#define APP_FB_PERIOD_MS 20u
#endif
#ifndef APP_ENC_DIV
/* 每几拍查询一次编码器；2 拍 = 4ms。 */
#define APP_ENC_DIV 2u
#endif
#ifndef APP_IDLE_RESEND_MS
/* 电调空闲状态命令的重发间隔，单位毫秒。 */
#define APP_IDLE_RESEND_MS 20u
#endif
#ifndef APP_VEL_LIMIT
/* 配置给电调的速度限值，单位 turns/s（转子侧）；10 turns/s ÷ 减速比 8 = 75 rpm（输出侧，上位机显示值）。 */
#define APP_VEL_LIMIT 10.0f
#endif
#ifndef APP_CUR_LIMIT
/* 配置给电调的电流限值，单位 A；每次使能经 mwSHS_set_limits 下发，会覆盖上位机设置。20 A × Kt 0.183 Nm/A × 减速比 8 ≈ 29 Nm（关节侧），整机限幅由 app_legs 的 tau_max 负责，此值仅作后备。 */
#define APP_CUR_LIMIT 20.0f
#endif
#ifndef APP_TRAJ_VEL
/* 位置轨迹最大速度，单位 turns/s。 */
#define APP_TRAJ_VEL 5.0f
#endif
#ifndef APP_TRAJ_ACC
/* 位置轨迹加速度和减速度，单位 turns/s^2。 */
#define APP_TRAJ_ACC 10.0f
#endif
#ifndef APP_TRAJ_INERTIA
/* 位置轨迹惯量前馈参数，单位由电调协议定义。 */
#define APP_TRAJ_INERTIA 0.0f
#endif

typedef struct {
    uint8_t bus;
    uint8_t node;
} motor_t;
/* 控制的电机所在 CAN 总线和节点号；g_motorSHS[i] 与 s_ctx[i] 和 motors[i] 一一对应。 */
static const motor_t motors[] =
{
    {0u, 1u},
    {0u, 2u},
    {0u, 3u},
    {0u, 4u},
    {1u,1u},
    {1u,2u},
    {1u,3u},
    {1u,4u},
};
_Static_assert(sizeof(motors) / sizeof(motors[0]) == APP_MOTORSHS_NUM,
               "motors must match APP_MOTOR_NUM");
/* 下标由 mode_index 转换；每行依次是输入模式、目标下限和上限；目标单位依次为 Nm、turns/s、turns。 */
static const struct {
    uint8_t input;
    float lo;
    float hi;
} modes[] = {{MWSHS_INPUT_DIRECT, -1.0f, 1.0f},
             {MWSHS_INPUT_DIRECT, -10.0f, 10.0f},
             {MWSHS_INPUT_TRAP, -20.0f, 20.0f}};
_Static_assert(sizeof(modes) / sizeof(modes[0]) == APP_SHS_MODE_POS - APP_SHS_MODE_TORQUE + 1,
               "modes must match app_mode_t");
volatile app_motorSHS_state_t g_motorSHS[APP_MOTORSHS_NUM]; /* Live Watch 和上层读写的公开状态。 */
volatile uint8_t g_motorSHS_kill;
/* 记录器只读取非 volatile 影子数组，不改变电机状态机与 HAL 超时时基。 */
static float s_motor_target[APP_MOTORSHS_NUM];
static uint8_t s_motor_phase[APP_MOTORSHS_NUM];
static uint8_t s_motor_fault[APP_MOTORSHS_NUM];
static const sim_var_desc_t s_motor_vars[] = {
    {"target", SIM_VAR_F32, APP_MOTORSHS_NUM, s_motor_target},
    {"phase", SIM_VAR_U8, APP_MOTORSHS_NUM, s_motor_phase},
    {"fault", SIM_VAR_U8, APP_MOTORSHS_NUM, s_motor_fault},
};
const sim_vars_t g_motorSHS_vars = {"motor", s_motor_vars, sizeof(s_motor_vars) / sizeof(s_motor_vars[0])};

/* log_snapshot：发布全部电机本拍目标、阶段与故障，供通用记录器搬运。 */
static void log_snapshot(void) {
    for (uint8_t i = 0u; i < APP_MOTORSHS_NUM; i++) {
        s_motor_target[i] = g_motorSHS[i].target;
        s_motor_phase[i] = (uint8_t)g_motorSHS[i].phase;
        s_motor_fault[i] = (uint8_t)g_motorSHS[i].fault;
    }
}

static uint32_t s_estop_t;  /* 上次群发急停时间，单位毫秒。 */
static uint8_t s_estop_on;  /* 本次失能是否已群发过。 */
/* 电机状态机内部上下文；跨 tick 保持，只在本文件内可见。 */
typedef struct {
    uint32_t last_tick;     /* 上一次执行状态机的时间，单位毫秒。 */
    uint32_t last_rx;       /* 最近收到新反馈帧的时间，单位毫秒。 */
    uint32_t last_fb;       /* 上一次发送反馈查询的时间，单位毫秒。 */
    uint32_t last_idle;     /* 上一次发送停机指令的时间，单位毫秒。 */
    uint32_t t0;            /* 当前闭环或模式切换等待阶段的起始时间，单位毫秒。 */
    uint32_t rx_count;      /* 上一次观察到的协议层接收帧计数。 */
    uint8_t need_rearm;     /* 必须先让 enable 回到 0，才能接受下一次 enable=1。 */
    uint8_t fb_index;       /* 下一条轮询反馈查询命令在 cmd[] 中的索引。 */
    uint8_t tick_cnt;       /* 本电机已执行的拍数，用于错开编码器查询 */
    app_motorSHS_phase_t phase;      /* 内部阶段，结束时同步到对应 g_motorSHS[i].phase。 */
    app_motorSHS_mode_t active_mode; /* 当前电调配置模式，结束时同步到对应 g_motorSHS[i].active_mode。 */
    app_motorSHS_mode_t pending_mode;/* 模式切换期间等待配置的新模式。 */
    app_motorSHS_fault_t fault;      /* 内部故障原因，结束时同步到对应 g_motorSHS[i].fault。 */
} motor_ctx_t;
/* 只由 app_motorSHS_init 和 app_motorSHS_tick 直接访问，其余函数通过参数取得。 */
static motor_ctx_t s_ctx[APP_MOTORSHS_NUM];

/* 向全部电机群发急停，不论在线与否。 */
static void estop_all(void) {
    for (uint8_t i = 0u; i < APP_MOTORSHS_NUM; i++)
        mwSHS_estop(motors[i].bus, motors[i].node);
}

/* 判断应用模式值是否对应 modes 表中的一项，防止非法索引。 */
static int valid_mode(app_motorSHS_mode_t mode) {
    return (unsigned)mode >= (unsigned)APP_SHS_MODE_TORQUE && (unsigned)mode <= (unsigned)APP_SHS_MODE_POS;
}
/* 合法模式转换为 modes 表下标；调用前必须通过 valid_mode 检查。 */
static unsigned mode_index(app_motorSHS_mode_t mode) {
    return (unsigned)mode - (unsigned)APP_SHS_MODE_TORQUE;
}
/* NaN 按 0 处理；其余值（包括 ±Inf）截到 [lo, hi]。 */
static float clamp_target(float x, float lo, float hi) {
    if (isnan(x))
        return 0.0f;
    return x < lo ? lo : (x > hi ? hi : x);
}
/* send_motion：按当前模式把目标转换成对应的电调运动指令
 * 参数：b 为总线号，n 为节点号，mode 为模式，x 为该模式单位下的目标
 * 说明：只在闭环运行阶段调用；发送前限幅，避免超出应用配置范围。
 */
static void send_motion(uint8_t b, uint8_t n, app_motorSHS_mode_t mode, float x) {
    if (!valid_mode(mode))
        return;
    float v = clamp_target(x, modes[mode_index(mode)].lo, modes[mode_index(mode)].hi);
    switch (mode) {
    case APP_SHS_MODE_TORQUE:
        mwSHS_torque(b, n, v);
        break;
    case APP_SHS_MODE_VEL:
        mwSHS_vel(b, n, v, 0.0f);
        break;
    case APP_SHS_MODE_POS:
        mwSHS_pos(b, n, v, 0, 0);
        break;
    default:
        break;
    }
}
/* 用零力矩输入喂电调轴级看门狗；在位置/速度模式下仅把力矩前馈置 0，不产生运动；电调空闲时只更新输入值，不会使能。 */
static void feed_watchdog(uint8_t b, uint8_t n) {
    mwSHS_torque(b, n, 0.0f);
}
/* configure：按给定模式依序配置电调并请求进入闭环
 * 参数：b 为总线号，n 为节点号，mode 为目标控制模式
 * 说明：仅在首次使能或模式切换时调用；先喂狗再清旧错误，再设控制/输入模式和限值，位置模式额外设轨迹参数，最后再次喂狗并进入闭环，确保工作状态使用完整配置。
 */
static void configure(uint8_t b, uint8_t n, app_motorSHS_mode_t mode) {
    if (!valid_mode(mode))
        return;
    /* 先喂狗再清除电调记录的旧故障，避免看门狗计数为 0 时立即再次超时。 */
    feed_watchdog(b, n);
    mwSHS_clear_errors(b, n);
    /* 选择控制量类型和输入解释方式，确定后续目标命令的含义。 */
    mwSHS_set_mode(b, n, mode, modes[mode_index(mode)].input);
    /* 先设定速度、电流保护限值，再允许电机进入闭环。 */
    mwSHS_set_limits(b, n, APP_VEL_LIMIT, APP_CUR_LIMIT);
    if (mode == APP_SHS_MODE_POS) {
        /* 位置轨迹模式需要额外设定运动速度、加减速度和惯量前馈。 */
        mwSHS_set_traj_vel(b, n, APP_TRAJ_VEL);
        mwSHS_set_traj_accel(b, n, APP_TRAJ_ACC, APP_TRAJ_ACC);
        mwSHS_set_traj_inertia(b, n, APP_TRAJ_INERTIA);
    }
    /* 所有参数已下发后再次喂狗，防止配置帧排队期间超时，再进入闭环。 */
    feed_watchdog(b, n);
    mwSHS_set_state(b, n, MWSHS_STATE_CLOSED_LOOP);
}
/* keep_idle：在电调尚未空闲时按间隔重复请求空闲状态
 * 参数：c 为电机上下文，b 为总线号，n 为节点号，now 为当前毫秒计时，state 为反馈状态
 * 说明：IDLE 和 FAULT 阶段每个 tick 调用；state 不是 IDLE 时，每 APP_IDLE_RESEND_MS 重发一次停机指令，防止单帧丢失导致电机按最后一条指令继续转（速度模式会一直转）。
 */
static void keep_idle(motor_ctx_t *c, uint8_t b, uint8_t n, uint32_t now, uint8_t state) {
    if (state != MWSHS_STATE_IDLE && (uint32_t)(now - c->last_idle) >= APP_IDLE_RESEND_MS) {
        mwSHS_set_state(b, n, MWSHS_STATE_IDLE);
        c->last_idle = now;
    }
}
/* request_feedback：错开编码器查询，并低频轮询电流和母线电压
 * 参数：c 为电机上下文，b 为总线号，n 为节点号，now 为当前毫秒计时
 * 说明：编码器每 APP_ENC_DIV 拍查询一次，电流和电压每 APP_FB_PERIOD_MS 交替查询。
 * 默认每条总线每拍约 4 力矩帧 + 2 编码器请求 + 2 应答；低频查询和心跳另计。
 */
static void request_feedback(motor_ctx_t *c, uint8_t b, uint8_t n, uint32_t now) {
    static const uint8_t cmd[] = {MWSHS_CMD_IQ, MWSHS_CMD_BUS};
    c->tick_cnt++;
    if (c->tick_cnt % APP_ENC_DIV == 0u)
        mwSHS_request(b, n, MWSHS_CMD_ENCODER);
    if ((uint32_t)(now - c->last_fb) >= APP_FB_PERIOD_MS) {
        mwSHS_request(b, n, cmd[c->fb_index]);
        c->fb_index ^= 1u;
        c->last_fb = now;
    }
}
/* enter_fault：锁存故障并请求电调空闲
 * 参数：c 为电机上下文，fault 为故障原因，b 为总线号，n 为节点号
 * 说明：故障后停止发送运动目标并要求 enable 先回零；回零后回到 IDLE，重新置位 enable 才会再次配置，成功进入 RUN 时清除故障。
 */
static void enter_fault(motor_ctx_t *c, app_motorSHS_fault_t fault, uint8_t b, uint8_t n) {
    c->fault = fault;
    mwSHS_set_state(b, n, MWSHS_STATE_IDLE);
    c->last_idle = c->last_tick;
    c->need_rearm = 1u;
    c->phase = APP_SHS_PH_FAULT;
}

/* app_motorSHS_init：清空公开状态并初始化状态机
 * 说明：系统启动时调用一次；使能需经历先为零再置一的重新使能过程。
 */
void app_motorSHS_init(void) {
    mwSHS_init(&s_can_ops);
    bsp_can_set_rx_callback(BSP_CAN_BUS1, shs_rx);
    bsp_can_set_rx_callback(BSP_CAN_BUS2, shs_rx);
    memset((void *)g_motorSHS, 0, sizeof(g_motorSHS));
    memset((void *)s_ctx, 0, sizeof(s_ctx));
    g_motorSHS_kill = 0u;
    s_estop_t = 0u;
    s_estop_on = 0u;
    for (uint8_t i = 0u; i < APP_MOTORSHS_NUM; i++) {
        /* 默认选力矩模式，这样上电后直接置 enable 不会因为 mode=0 报错。 */
        g_motorSHS[i].mode = APP_SHS_MODE_TORQUE;
        s_ctx[i].need_rearm = 1u;
        s_ctx[i].tick_cnt = (uint8_t)i;
        s_ctx[i].phase = APP_SHS_PH_OFFLINE;
        s_ctx[i].active_mode = APP_SHS_MODE_TORQUE;
        s_ctx[i].pending_mode = APP_SHS_MODE_TORQUE;
        s_ctx[i].fault = APP_SHS_FAULT_NONE;
    }
    log_snapshot();
}

/* app_motorSHS_kill：置位一键失能标志
 * 说明：只写标志不发帧，可在任意上下文（含按键中断）调用。
 */
void app_motorSHS_kill(void) {
    g_motorSHS_kill = 1u;
}

/* app_motorSHS_kill_release：清除一键失能标志
 * 说明：只写标志，可由主循环或 TIM6 内的 app_rc 调用；解除后仍需 enable 先回零再置一。
 */
void app_motorSHS_kill_release(void) {
    g_motorSHS_kill = 0u;
}

/* motor_step：推进单个电机的状态机
 * 计时：由 app_motorSHS_tick 传入同一拍的平台单调毫秒计时，可自然回绕。
 * 参数：c 为电机内部上下文，a 为对应公开状态，mt 为电机总线和节点配置，now 为当前毫秒计时，cfg_slot 为本拍剩余配置名额。
 * 说明：负责单个电机的通信超时、反馈查询、状态配置和目标发送。
 * 状态转移表：
   *   当前阶段          条件                                      下一阶段
   *   任意              g_motorSHS_kill=1                             FAULT(KILLED)，期间忽略 enable
   *   任意(非OFFLINE)   200ms 内没有新帧                           OFFLINE
   *   OFFLINE           近期收到帧                                IDLE（需要重新使能）
   *   IDLE              enable 先为 0 再为 1，且 mode 允许，且本拍配置名额未被占用   CONFIG → 同一拍进入 WAIT_CL
   *   IDLE              同上但 mode 非法                            FAULT(BAD_MODE)
   *   WAIT_CL           心跳 state==8                              RUN
   *   WAIT_CL           enable=0                                   IDLE
   *   WAIT_CL           超时                                       FAULT(CL_TIMEOUT)
   *   RUN               enable=0                                   IDLE
   *   RUN               axis_error 非 0                            FAULT(AXIS_ERROR)
   *   RUN               state 不再是 8                             FAULT(LOST_CL)
   *   RUN               mode 改变且非法                            FAULT(BAD_MODE)
   *   RUN               mode 改变且允许                            SWITCH
   *   SWITCH            enable=0                                   IDLE
   *   SWITCH            心跳 state==1，且本拍配置名额未被占用     CONFIG → 同一拍进入 WAIT_CL
   *   SWITCH            超时                                       FAULT(SWITCH_TIMEOUT)
   *   FAULT             enable=0                                   IDLE
   *   FAULT             enable 非 0                                FAULT
   *   IDLE 和 FAULT 期间，只要电机没回到空闲，每 20ms 重发一次停机指令。
   *   这样可防止单帧丢失后电机继续执行最后一条指令，速度模式尤其会持续转动。
   */
static void motor_step(motor_ctx_t *c, volatile app_motorSHS_state_t *a, const motor_t *mt, uint32_t now,
                       uint8_t *cfg_slot) {
    uint8_t b = mt->bus, n = mt->node;
    mwSHS_motor_t m;
    app_motorSHS_mode_t mode;
    uint8_t enable;
    c->last_tick = now;
    /* 每次调度只采样一次输入，避免一次状态推进中混用调试器刚改写的值。 */
    mode = a->mode;
    enable = a->enable;
    mwSHS_snapshot(b, n, &m);
    memcpy((void *)&a->fb, &m, sizeof(m));
    /* rx_count 变化就刷新最近收帧时间；rx_count 非零只说明曾经收到过帧，不代表现在在线。 */
    if (m.rx_count != c->rx_count) {
        c->rx_count = m.rx_count;
        c->last_rx = now;
    }
    if (g_motorSHS_kill) {
        c->phase = APP_SHS_PH_FAULT;
        c->fault = APP_SHS_FAULT_KILLED;
        c->need_rearm = 1u;
        goto finish;
    }
    if (c->phase != APP_SHS_PH_OFFLINE && (uint32_t)(now - c->last_rx) > APP_RX_TIMEOUT_MS)
        c->phase = APP_SHS_PH_OFFLINE;
    if (c->phase == APP_SHS_PH_OFFLINE) {
        if (m.rx_count != 0u && (uint32_t)(now - c->last_rx) <= APP_RX_TIMEOUT_MS) {
            c->phase = APP_SHS_PH_IDLE;
            c->need_rearm = 1u;
            /* 把上次重发时间往前拨一个周期，让刚进入 IDLE 时 keep_idle 立即发一次停机指令。 */
            c->last_idle = now - APP_IDLE_RESEND_MS;
        } else {
            /* 离线时什么都不发，直接跳到末尾同步对应 g_motorSHS[i]。 */
            goto finish;
        }
    }
    request_feedback(c, b, n, now);
    switch (c->phase) {
    case APP_SHS_PH_IDLE:
        /* 保持电调空闲；必须先观察到 enable=0，才能接受新的 enable=1 请求。 */
        keep_idle(c, b, n, now, m.state);
        if (!enable) {
            c->need_rearm = 0u;
            break;
        }
        if (c->need_rearm)
            break;
        if (!valid_mode(mode)) {
            enter_fault(c, APP_SHS_FAULT_BAD_MODE, b, n);
            break;
        }
        if (!*cfg_slot)
            break;
        *cfg_slot = 0u;
        c->phase = APP_SHS_PH_CONFIG;
        configure(b, n, mode);
        c->active_mode = mode;
        c->t0 = now;
        c->phase = APP_SHS_PH_WAIT_CL;
        break;
    case APP_SHS_PH_WAIT_CL:
        /* 等待电调状态反馈确认闭环；失能立即退回空闲，等待超时则锁存故障。 */
        if (!enable) {
            mwSHS_set_state(b, n, MWSHS_STATE_IDLE);
            c->phase = APP_SHS_PH_IDLE;
            c->last_idle = now;
            break;
        }
        /* 闭环确认（心跳）到达之前持续喂狗，防止在 RUN 开始发运动指令前超时。 */
        feed_watchdog(b, n);
        if (m.state == MWSHS_STATE_CLOSED_LOOP) {
            if (c->active_mode == APP_SHS_MODE_POS)
                /* 位置模式从反馈位置起步，避免使能时跳向上次目标。 */
                a->target = m.pos;
            c->fault = APP_SHS_FAULT_NONE;
            c->phase = APP_SHS_PH_RUN;
        } else if ((uint32_t)(now - c->t0) > APP_CL_TIMEOUT_MS) {
            mwSHS_request(b, n, MWSHS_CMD_ERROR);
            enter_fault(c, APP_SHS_FAULT_CL_TIMEOUT, b, n);
        }
        break;
    case APP_SHS_PH_RUN:
        /* 闭环期间持续检查使能、轴错误和闭环反馈；模式变化时先退出再重新配置。 */
        if (!enable) {
            mwSHS_set_state(b, n, MWSHS_STATE_IDLE);
            c->phase = APP_SHS_PH_IDLE;
            c->last_idle = now;
            break;
        }
        if (m.axis_error != 0u) {
            enter_fault(c, APP_SHS_FAULT_AXIS_ERROR, b, n);
            break;
        }
        if (m.state != MWSHS_STATE_CLOSED_LOOP) {
            enter_fault(c, APP_SHS_FAULT_LOST_CL, b, n);
            break;
        }
        if (mode != c->active_mode) {
            if (!valid_mode(mode))
                enter_fault(c, APP_SHS_FAULT_BAD_MODE, b, n);
            else {
                c->pending_mode = mode;
                mwSHS_set_state(b, n, MWSHS_STATE_IDLE);
                c->t0 = now;
                c->phase = APP_SHS_PH_SWITCH;
            }
            break;
        }
        send_motion(b, n, c->active_mode, a->target);
        break;
    case APP_SHS_PH_SWITCH:
        /* 等待电调确认已空闲，再应用待切换模式；状态迟迟不变则报告切换超时。 */
        if (!enable) {
            mwSHS_set_state(b, n, MWSHS_STATE_IDLE);
            c->phase = APP_SHS_PH_IDLE;
            c->last_idle = now;
            break;
        }
        /* 闭环确认（心跳）到达之前持续喂狗，防止在 RUN 开始发运动指令前超时。 */
        feed_watchdog(b, n);
        if (m.state == MWSHS_STATE_IDLE) {
            if (*cfg_slot) {
                *cfg_slot = 0u;
                c->phase = APP_SHS_PH_CONFIG;
                configure(b, n, c->pending_mode);
                c->active_mode = c->pending_mode;
                c->t0 = now;
                c->phase = APP_SHS_PH_WAIT_CL;
            }
        } else if ((uint32_t)(now - c->t0) > APP_CL_TIMEOUT_MS)
            enter_fault(c, APP_SHS_FAULT_SWITCH_TIMEOUT, b, n);
        break;
    case APP_SHS_PH_FAULT:
        /* 故障期间持续维持空闲；enable 回零后解除重新使能门槛。故障阶段也按周期重发停机指令。 */
        keep_idle(c, b, n, now, m.state);
        if (!enable) {
            c->phase = APP_SHS_PH_IDLE;
            c->need_rearm = 0u;
        }
        break;
    default:
        break;
    }
finish:
    a->phase = c->phase;
    a->active_mode = c->active_mode;
    a->fault = c->fault;
}

/* app_motorSHS_tick：采集反馈并推进全部电机控制状态机
 * 计时：内部通过 bsp_millis 获取单调毫秒计时，可自然回绕。
 * 说明：由调度器按 APP_MOTORSHS_PERIOD_MS 周期调用；每次调用依次推进全部电机的状态机。失能时先群发急停。
 * 每拍最多配置一个电机：一次配置最多 9 帧，多台同时使能会挤爆 32 级发送 FIFO；最坏 8 台依次配置也只延迟 16ms。
 */
void app_motorSHS_tick(void) {
    uint32_t now = bsp_millis(); /* 同一拍内所有电机使用同一时间基准。 */
    uint8_t cfg_slot = 1u;
    if (g_motorSHS_kill) {
        if (!s_estop_on || (uint32_t)(now - s_estop_t) >= APP_IDLE_RESEND_MS) {
            estop_all();
            s_estop_t = now;
            s_estop_on = 1u;
        }
    } else {
        s_estop_on = 0u;
    }
    for (uint8_t i = 0u; i < APP_MOTORSHS_NUM; i++)
        motor_step(&s_ctx[i], &g_motorSHS[i], &motors[i], now, &cfg_slot);
    log_snapshot();
}

/* g_motor_all_running：仅当八台电机全部处于 RUN 时返回 1。 */
uint8_t g_motor_all_running(void) {
    for (uint8_t i = 0u; i < APP_MOTORSHS_NUM; i++) {
        if (g_motorSHS[i].phase != APP_SHS_PH_RUN)
            return 0u;
    }
    return 1u;
}

void motor_control_test(void)
{
    app_motorSHS_tick();
}
