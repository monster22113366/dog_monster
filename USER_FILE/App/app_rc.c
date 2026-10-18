/*
 * rc全称 Remote Control，即遥控。
 * 本文件属于 App 应用层，组合 bsp_sbus 接收与 sbus 算法实现遥控策略。
 * 依赖 app_ctrl 控制时基、app_legs 整机命令、app_motorSHS 急停和标准内存函数。
 * app_rc 与 app_legs 在同一 TIM6 上下文顺序执行，写 g_legs 无并发竞争。
 * enable=0 不接管台架；使能后交出轮控所有权，上线/重连只同步模式开关。
 */
#include "app_rc.h"
#include "bsp_sbus.h"
#include "sbus.h"
#include "app_ctrl.h"
#include "app_legs.h"
#include "app_chassis.h"
#include "app_motorSHS.h"
#include "app_motorQZB.h"
#include <string.h>

_Static_assert(APP_RC_CH_FWD < SBUS_CH_NUM && APP_RC_CH_MODE < SBUS_CH_NUM &&
               APP_RC_CH_ESTOP < SBUS_CH_NUM && APP_RC_CH_ZERO < SBUS_CH_NUM &&
               APP_RC_CH_WHEEL < SBUS_CH_NUM && APP_RC_CH_YAW < SBUS_CH_NUM &&
               APP_RC_CH_LEG < SBUS_CH_NUM && APP_RC_CH_RESET < SBUS_CH_NUM,
               "RC channel index must fit SBUS");
_Static_assert(APP_RC_SWITCH_STABLE > 0u && APP_RC_SWITCH_STABLE <= UINT32_MAX,
               "RC switch stability must be a positive frame count");

volatile rc_debug_t g_rc;
static struct {
    uint32_t seq, last_ms, stable_frames, last_retry_ms;
    uint32_t last_good_ms, reset_first_ms, loss_ms;
    uint8_t loss_release_pending, loss_released;
    uint8_t good_valid, reset_state;
    uint8_t received, failsafe, online, ever_online, loss_handled, enabled;
    uint8_t synced_mode, candidate_mode, requested;
    uint8_t synced_zero, zero_candidate, zero_timing;
    uint32_t zero_start_ms;
    uint32_t wheel_stable_frames;
    uint8_t synced_wheel, candidate_wheel, wheel_requested;
    uint8_t last_leg_mode, stride_armed, chassis_owned;
} s_rc;
static uint8_t s_estop_by_rc;

/* sw2：ET07 SA 两挡约为 353（低）/1695（高），按 1024 分界。 */
static uint8_t sw2(uint16_t raw) {
    return raw < 1024u ? 0u : 1u;
}

/* 记录器只读取非 volatile 影子变量，由本任务末尾统一发布。 */
static struct {
    uint8_t online, sw_legs, estop, sw_wheel;
    float stride_cmd;
} s_rc_log;
static const sim_var_desc_t s_rc_vars[] = {
    {"online", SIM_VAR_U8, 0u, &s_rc_log.online},
    {"sw_mode", SIM_VAR_U8, 0u, &s_rc_log.sw_legs},
    {"estop", SIM_VAR_U8, 0u, &s_rc_log.estop},
    {"stride_cmd", SIM_VAR_F32, 0u, &s_rc_log.stride_cmd},
    {"sw_wheel", SIM_VAR_U8, 0u, &s_rc_log.sw_wheel},
};
const sim_vars_t g_rc_vars = {"rc", s_rc_vars, sizeof(s_rc_vars) / sizeof(s_rc_vars[0])};

/* log_snapshot：复制接收统计与记录影子量，避免向记录器暴露 volatile 指针。 */
static void log_snapshot(void) {
    uint32_t bad, err;
    bsp_sbus_stats(&bad, &err);
    g_rc.bad_count = bad;
    g_rc.err_count = err;
    s_rc_log.online = g_rc.online;
    s_rc_log.sw_legs = g_rc.sw_legs;
    s_rc_log.estop = g_rc.estop;
    s_rc_log.stride_cmd = g_rc.stride_cmd;
    s_rc_log.sw_wheel = g_rc.sw_wheel;
}

/* app_rc_init：复位内部状态及调试接口，不修改整机命令或电机急停。 */
void app_rc_init(void) {
    memset((void *)&g_rc, 0, sizeof(g_rc));
    memset(&s_rc, 0, sizeof(s_rc));
    s_rc.requested = 0xffu;
    s_rc.wheel_requested = 0xffu;
    s_estop_by_rc = 0u;
    g_rc.enable = APP_RC_ENABLE_DEFAULT;
    g_rc.age_ms = UINT32_MAX;
    g_rc.good_age_ms = UINT32_MAX;
    g_rc.init_err = (int8_t)bsp_sbus_init();
    log_snapshot();
}

/* mode_command：把 SB 三挡档位转换为整机命令。 */
static uint8_t mode_command(uint8_t mode) {
    static const uint8_t commands[3] = {LEGS_PASSIVE, LEGS_STAND, LEGS_TROT};
    return commands[mode];
}

/* 在腿任务接受 TROT 前清步长，避免两次遥控采样之间沿用上次小跑命令。 */
static void leg_command(uint8_t command) {
    if (command == LEGS_TROT && g_legs.mode != LEGS_TROT) {
        s_rc.stride_armed = 0u;
        g_legs.stride = 0.0f;
    }
    g_legs.cmd = command;
}

/* mode_update：只对新收到的有效帧计数，稳定档位变化才写一次命令。
 * app_legs 只在 STAND 渐变结束后接受 TROT；记录 requested 供后续重发。
 * 同档位期间不覆盖 Live Watch 手动命令；恢复在线时由调用方重新同步基准档位。
 */
static void mode_update(uint8_t mode) {
    if (mode != s_rc.candidate_mode) {
        s_rc.candidate_mode = mode;
        s_rc.stable_frames = 1u;
    } else if (s_rc.stable_frames < APP_RC_SWITCH_STABLE) {
        s_rc.stable_frames++;
    }
    if (s_rc.stable_frames >= APP_RC_SWITCH_STABLE && mode != s_rc.synced_mode) {
        s_rc.synced_mode = mode;
        s_rc.requested = mode_command(mode);
        leg_command(s_rc.requested);
        s_rc.last_retry_ms = app_ctrl_ms();
    }
}

/* SC 与 SB 一样只消费新有效帧；档位与已发布请求分开保存。 */
static void wheel_update(uint8_t mode) {
    if (mode != s_rc.candidate_wheel) {
        s_rc.candidate_wheel = mode;
        s_rc.wheel_stable_frames = 1u;
    } else if (s_rc.wheel_stable_frames < APP_RC_SWITCH_STABLE) {
        s_rc.wheel_stable_frames++;
    }
    if (s_rc.wheel_stable_frames >= APP_RC_SWITCH_STABLE && mode != s_rc.synced_wheel) {
        static const uint8_t commands[3] = {CHASSIS_OFF, CHASSIS_HOLD, CHASSIS_DRIVE};
        s_rc.synced_wheel = mode;
        s_rc.wheel_requested = commands[mode];
        g_chassis.mode_req = s_rc.wheel_requested;
    }
}

/* 左油门进入小跑后必须拉到底一次；轮速仅跟随已发布的 DRIVE。 */
static void analog_update(void) {
    const uint8_t drive = s_rc.wheel_requested == CHASSIS_DRIVE;
    g_chassis.fwd = drive ? sbus_norm(g_rc.ch[APP_RC_CH_FWD], APP_RC_DEADBAND) : 0.0f;
    g_chassis.yaw = drive ? sbus_norm(g_rc.ch[APP_RC_CH_YAW], APP_RC_DEADBAND) : 0.0f;
    if (g_legs.mode == LEGS_STAND) {
        const float norm = sbus_norm(g_rc.ch[APP_RC_CH_LEG], 0.0f);
        g_legs.stand_z = LEGS_Z_MIN + (norm + 1.0f) * 0.5f * (LEGS_Z_MAX - LEGS_Z_MIN);
    } else if (g_legs.mode == LEGS_TROT) {
        const float norm = sbus_norm(g_rc.ch[APP_RC_CH_LEG], 0.0f);
        float u = 0.0f;
        if (norm <= APP_RC_STRIDE_ZERO_NORM)
            s_rc.stride_armed = 1u;
        else
            u = (norm + 1.0f) * 0.5f;
        g_rc.stride_cmd = s_rc.stride_armed ? u * APP_RC_STRIDE_MAX : 0.0f;
        g_legs.stride = g_rc.stride_cmd;
    }
}

/* 使能/重连与复位共用：只同步 SB/SC，不发布模式，不改零位或急停状态。 */
static void sync_switches(uint8_t fresh) {
    if (fresh) {
        s_rc.synced_mode = s_rc.candidate_mode = g_rc.sw_legs;
        s_rc.stable_frames = 0u;
        s_rc.synced_wheel = s_rc.candidate_wheel = g_rc.sw_wheel;
        s_rc.wheel_stable_frames = 0u;
    }
    s_rc.requested = s_rc.wheel_requested = 0xffu;
    s_rc.stride_armed = 0u;
}

static void reset_all(void) {
    s_rc.loss_released = 0u;
    s_rc.loss_release_pending = 0u;
    leg_command(LEGS_PASSIVE);
    g_legs.stride = 0.0f;
    g_chassis.fwd = g_chassis.yaw = 0.0f;
    g_chassis.mode_req = CHASSIS_OFF;
    app_motorQZB_request_clear();
    sync_switches(1u);
}

/* 仅新好帧推进；坏帧立即丢弃未完成手势，避免旧通道值拼接。 */
static uint8_t reset_gesture(uint32_t now) {
    const uint16_t ch = g_rc.ch[APP_RC_CH_RESET];
    if (s_rc.reset_state >= 2u &&
        (uint32_t)(now - s_rc.reset_first_ms) > APP_RC_RESET_WINDOW_MS) {
        s_rc.reset_state = ch <= APP_RC_RESET_LEFT ? 1u : 0u;
        return 0u;
    }
    switch (s_rc.reset_state) {
    case 0u:
        if (ch <= APP_RC_RESET_LEFT) s_rc.reset_state = 1u;
        break;
    case 1u:
        if (ch >= APP_RC_RESET_RIGHT) {
            s_rc.reset_state = 2u;
            s_rc.reset_first_ms = now;
        }
        break;
    case 2u:
        if (ch <= APP_RC_RESET_LEFT) s_rc.reset_state = 3u;
        break;
    case 3u:
        if (ch >= APP_RC_RESET_RIGHT) {
            s_rc.reset_state = 0u;
            g_rc.reset_count++;
            return 1u;
        }
        break;
    default: s_rc.reset_state = 0u; break;
    }
    return 0u;
}

/* app_rc_tick：每 10 ms 读取完整帧并执行遥控策略，超时用无符号差支持时基回绕。 */
void app_rc_tick(void) {
    uint8_t raw[SBUS_FRAME_LEN];
    sbus_frame_t frame;
    uint8_t new_frame = 0u, good_frame = 0u;
    const uint32_t seq = bsp_sbus_read(raw);
    const uint32_t now = app_ctrl_ms();
    const uint8_t enabled = g_rc.enable != 0u;
    const uint8_t enable_rise = enabled && !s_rc.enabled;
    const uint8_t enable_fall = !enabled && s_rc.enabled;
    const uint8_t was_online = s_rc.online;
    if (seq != s_rc.seq && sbus_decode(raw, &frame) == 0) {
        s_rc.seq = seq;
        s_rc.last_ms = now;
        s_rc.received = 1u;
        s_rc.failsafe = frame.failsafe;
        new_frame = 1u;
        g_rc.frames++;
        for (uint8_t i = 0u; i < SBUS_CH_NUM; i++)
            g_rc.ch[i] = frame.ch[i];
        good_frame = !frame.lost && !frame.failsafe;
        if (good_frame) {
            s_rc.last_good_ms = now;
            s_rc.good_valid = 1u;
        }
        /* 单帧丢失不触发；连续丢帧超过 APP_RC_LOST_MS 才失联。 */
        g_rc.frame_lost += frame.lost;
        g_rc.sw_legs = sbus_switch3(frame.ch[APP_RC_CH_MODE]);
        g_rc.sw_wheel = sbus_switch3(frame.ch[APP_RC_CH_WHEEL]);
        g_rc.sw_zero = sw2(frame.ch[APP_RC_CH_ZERO]);
        const float estop = sbus_norm(frame.ch[APP_RC_CH_ESTOP], 0.0f);
        if (estop > APP_RC_ESTOP_ON)
            g_rc.estop = 1u;
        else if (estop < APP_RC_ESTOP_OFF)
            g_rc.estop = 0u;
    }

    g_rc.age_ms = s_rc.received ? (uint32_t)(now - s_rc.last_ms) : UINT32_MAX;
    g_rc.good_age_ms = s_rc.good_valid ? (uint32_t)(now - s_rc.last_good_ms) : UINT32_MAX;
    const uint8_t fresh = s_rc.received && g_rc.age_ms <= APP_RC_TIMEOUT_MS &&
                          !s_rc.failsafe && g_rc.good_age_ms <= APP_RC_LOST_MS;
    if (!fresh || (new_frame && !good_frame)) s_rc.reset_state = 0u;
    const uint8_t reset_trigger = new_frame && good_frame && fresh && reset_gesture(now);
    g_rc.online = fresh;
    if (fresh || enable_fall || enable_rise)
        s_rc.loss_release_pending = 0u;
    g_rc.stride_cmd = 0.0f;
    if (fresh) s_rc.ever_online = 1u;
    if (enabled) {
        g_chassis.enable = 1u;
        s_rc.chassis_owned = 1u;
    }
    if (enable_fall && s_rc.chassis_owned) {
        /* 只释放本模块接管过的底盘；后续监视拍保留台架手动输入。 */
        g_chassis.fwd = 0.0f;
        g_chassis.yaw = 0.0f;
        g_chassis.mode_req = CHASSIS_HOLD;
        g_chassis.enable = 0u;
        s_rc.chassis_owned = 0u;
    }
    if (g_legs.mode != s_rc.last_leg_mode || !enabled || !fresh)
        s_rc.stride_armed = 0u;

    if (enable_fall) {
        s_estop_by_rc = 0u;
        s_rc.loss_released = 0u;
    }

    if (enabled && reset_trigger) {
        reset_all();
    } else if (enabled && (enable_rise || (!was_online && fresh))) {
        if (!enable_rise && !was_online && fresh && s_rc.loss_released) {
            app_motorQZB_request_clear();
            s_rc.loss_released = 0u;
        }
        /* 使能沿或使能后的首次上线只同步档位，不下发命令，避免调试确认后立即动作。 */
        if (fresh) {
            s_rc.synced_zero = g_rc.sw_zero;
            s_rc.zero_timing = 0u;
        }
        sync_switches(fresh);
        g_chassis.fwd = g_chassis.yaw = 0.0f;
        s_rc.loss_handled = fresh ? 0u : 1u;
        if (fresh && g_legs.mode == LEGS_TROT) g_legs.stride = 0.0f;
        s_estop_by_rc = 0u;
    } else if (enabled && fresh) {
        if (new_frame) {
            mode_update(g_rc.sw_legs);
            wheel_update(g_rc.sw_wheel);
        }
        if (g_rc.sw_zero == s_rc.synced_zero) {
            s_rc.zero_timing = 0u;
        } else {
            if (!s_rc.zero_timing || s_rc.zero_candidate != g_rc.sw_zero) {
                s_rc.zero_candidate = g_rc.sw_zero;
                s_rc.zero_start_ms = now;
                s_rc.zero_timing = 1u;
            } else if ((uint32_t)(now - s_rc.zero_start_ms) >= APP_RC_ZERO_HOLD_MS) {
                /* app_legs 还会要求 PASSIVE 且 8 台电机在线；编码器无法判断腿是否真的折叠，
                 * 操作者须先把腿折叠好再拨 SA。条件不满足时本次拨动作废，不挂起。 */
                if (g_rc.sw_legs == 0u && g_legs.mode == LEGS_PASSIVE &&
                    g_rc.estop == 0u && g_motorSHS_kill == 0u) {
                    g_legs.zero_req = 1u;
                    g_rc.zero_count++;
                } else {
                    g_rc.zero_reject++;
                }
                s_rc.synced_zero = g_rc.sw_zero;
                s_rc.zero_timing = 0u;
            }
        }
        if (new_frame && g_rc.sw_legs == 2u &&
            s_rc.synced_mode == 2u && s_rc.requested == LEGS_TROT &&
            g_legs.mode == LEGS_STAND && (uint32_t)(now - s_rc.last_retry_ms) >= APP_RC_RETRY_MS) {
            /* app_legs 的 TROT 门控会改写未接受的命令；若只在拨动沿写一次，快速拨档会丢请求。
             * 只在开关仍为高档时重试，不会覆盖 Live Watch 对其他模式的写入。
             */
            leg_command(LEGS_TROT);
            s_rc.last_retry_ms = now;
        }
        s_rc.loss_handled = 0u;
        analog_update();

        const float estop = sbus_norm(g_rc.ch[APP_RC_CH_ESTOP], 0.0f);
        if (estop > APP_RC_ESTOP_ON) {
            /* 已有的 Live Watch 急停不归遥控所有，不能在松开 SD 时擅自解除。 */
            if (g_motorSHS_kill == 0u)
                s_estop_by_rc = 1u;
            app_motorSHS_kill();
        } else if (s_estop_by_rc && estop < APP_RC_ESTOP_OFF && g_rc.sw_legs == 0u) {
            /* 只解除本模块触发的急停，且要求在线、松开 SD、SB 回到低档。 */
            app_motorSHS_kill_release();
            s_estop_by_rc = 0u;
        }
    } else if (enabled && s_rc.ever_online && !s_rc.loss_handled) {
        /* 一次离线仅处理一次；收步后由小跑退回站立，不急停，避免落地瘫倒。 */
        g_legs.stride = 0.0f;
        g_chassis.fwd = g_chassis.yaw = 0.0f;
        g_chassis.mode_req = CHASSIS_HOLD;
        s_rc.wheel_requested = 0xffu;
        if (g_legs.mode == LEGS_TROT)
            g_legs.cmd = LEGS_STAND;
        g_rc.lost_count++;
        s_rc.loss_handled = 1u;
        s_rc.loss_ms = now;
        s_rc.loss_release_pending = 1u;
        s_rc.zero_timing = 0u;
    }
#if APP_RC_LOSS_RELEASE_MS != 0xFFFFFFFFu
    if (enabled && !fresh && s_rc.loss_release_pending
#if APP_RC_LOSS_RELEASE_MS != 0u
        && (uint32_t)(now - s_rc.loss_ms) >= APP_RC_LOSS_RELEASE_MS
#endif
    ) {
        g_chassis.mode_req = CHASSIS_OFF;
        s_rc.loss_released = 1u;
        s_rc.loss_release_pending = 0u;
    }
#endif
    /* 除关闭沿一次性释放自有底盘外，监视拍不写控制量；台架需保持 g_rc.enable=0。 */
    s_rc.last_leg_mode = g_legs.mode;
    s_rc.online = fresh;
    s_rc.enabled = enabled;
    g_rc.ever_online = s_rc.ever_online;
    log_snapshot();
}
