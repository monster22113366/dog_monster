/*
 * chassis 意为 底盘
 * 本文件属于 App 应用层，组合 wheel_mix 算法实现差速轮速混控。
 * 依赖 Bsp 定时器接口及 IMU、遥控、整机、电机、通用记录器任务，不直接访问硬件。
 * 各任务只在 TIM6 中断中执行；调试说明见 app_chassis.h 文件头。
 */
#include "app_chassis.h"
#include "app_legs.h"
#include "app_ctrl.h"
#include "wheel_mix.h"
#include "bsp_critical.h"
#include <math.h>
#include <string.h>

_Static_assert(APP_MOTORQZB_COUNT == 4u, "chassis requires four wheels");
_Static_assert(APP_CHASSIS_PERIOD_MS == APP_CTRL_PERIOD_MS, "chassis period mismatch");
/* GCC 常量折叠允许浮点参数的编译期约束。 */
__extension__ _Static_assert(APP_CHASSIS_HOLD_KD >= 0.0f &&
    APP_CHASSIS_HOLD_KD <= APP_MOTORQZB_VEL_KD_LIMIT &&
    APP_CHASSIS_DRIVE_KD >= 0.0f && APP_CHASSIS_DRIVE_KD <= APP_MOTORQZB_VEL_KD_LIMIT,
    "chassis kd exceeds motor limit");

/* 装机后实测确认，使车体前进为正。 */
static const float s_dir[4] = {-1.0f, 1.0f, -1.0f, 1.0f};
volatile chassis_debug_t g_chassis;
static float s_speed[4];
static uint8_t s_ready, s_latched, s_off_seen;
static uint8_t s_owned;
static struct {
    uint8_t mode, running_mask;
    float target[4], vel[4];
} s_log;
static const sim_var_desc_t s_vars[] = {
    {"mode", SIM_VAR_U8, 0u, &s_log.mode},
    {"target", SIM_VAR_F32, 4u, s_log.target},
    {"vel", SIM_VAR_F32, 4u, s_log.vel},
    {"running_mask", SIM_VAR_U8, 0u, &s_log.running_mask},
};
const sim_vars_t g_chassis_vars = {"chassis", s_vars, sizeof(s_vars) / sizeof(s_vars[0])};

static void log_snapshot(void) {
    s_log.mode = g_chassis.mode;
    s_log.running_mask = g_chassis.running_mask;
    for (uint8_t i = 0u; i < 4u; i++) {
        s_log.target[i] = g_chassis.target[i];
        s_log.vel[i] = g_chassis.vel[i];
    }
}

void app_chassis_init(void) {
    memset((void *)&g_chassis, 0, sizeof(g_chassis));
    memset(s_speed, 0, sizeof(s_speed));
    s_ready = s_latched = s_off_seen = 0u;
    s_owned = 0u;
    g_chassis.mode_req = CHASSIS_HOLD;
    g_chassis.vmax = APP_CHASSIS_VMAX;
    log_snapshot();
}

/* 首次全部在线前仅等待；锁存后 OFF 确认可在故障仍存在时记住。
 * 故障未消失时始终失能，不依赖遥控请求是否已回到 DRIVE。 */
static uint8_t effective_mode(uint8_t req, uint8_t legs_mode,
                              uint8_t bad, app_motorQZB_fault_t fault) {
    if (!s_ready && g_chassis.online_mask == 0x0fu)
        s_ready = 1u;
    if (s_ready && bad && !s_latched) {
        s_latched = 1u;
        s_off_seen = 0u;
        g_chassis.fault = fault;
    }
    if (s_latched) {
        if (req == CHASSIS_OFF)
            s_off_seen = 1u;
        else if (s_off_seen && !bad && g_chassis.online_mask == 0x0fu && req <= CHASSIS_DRIVE) {
            s_latched = s_off_seen = 0u;
            g_chassis.fault = APP_QZB_FAULT_NONE;
        }
    }
    if (!s_ready || s_latched ||
#if !APP_CHASSIS_IGNORE_LEGS
        legs_mode == LEGS_PASSIVE ||
#endif
        req > CHASSIS_DRIVE)
        return CHASSIS_OFF;
    if (!APP_CHASSIS_TROT_DRIVE && legs_mode == LEGS_TROT && req == CHASSIS_DRIVE)
        return CHASSIS_HOLD;
    return req;
}

void app_chassis_tick(void) {
    uint32_t key = bsp_critical_enter();
    const uint8_t enabled = g_chassis.enable;
    const uint8_t req = g_chassis.mode_req, legs_mode = g_legs.mode;
    wheel_cmd_t cmd = {g_chassis.fwd, g_chassis.yaw, g_chassis.vmax};
    float feedback[4];
    uint8_t bad = 0u;
    app_motorQZB_fault_t fault = APP_QZB_FAULT_NONE;
    g_chassis.online_mask = g_chassis.running_mask = 0u;
    for (uint8_t i = 0u; i < 4u; i++) {
        feedback[i] = g_motorQZB[i].fb.vel;
        const app_motorQZB_phase_t phase = g_motorQZB[i].phase;
        if (g_motorQZB[i].online) g_chassis.online_mask |= (uint8_t)(1u << i);
        if (phase == APP_QZB_RUN) g_chassis.running_mask |= (uint8_t)(1u << i);
        if (!bad && (phase == APP_QZB_FAULT || phase == APP_QZB_OFFLINE)) {
            bad = 1u;
            fault = phase == APP_QZB_OFFLINE ? APP_QZB_FAULT_OFFLINE : g_motorQZB[i].fault;
        }
    }
    const uint8_t mode = enabled ? effective_mode(req, legs_mode, bad, fault) : CHASSIS_OFF;
    g_chassis.mode = mode;
    /* 仅释放实际接管过的轮子一次，保留驱动现场和故障恢复状态。 */
    if (!enabled && s_owned) {
        for (uint8_t i = 0u; i < 4u; i++) {
            g_motorQZB[i].enable = 0u;
            s_speed[i] = 0.0f;
            g_chassis.target[i] = 0.0f;
        }
        s_owned = 0u;
    }
    bsp_critical_leave(key);
    for (uint8_t i = 0u; i < 4u; i++)
        g_chassis.vel[i] = feedback[i] * s_dir[i];
    if (!enabled) {
        log_snapshot();
        return;
    }

    if (!isfinite(cmd.vmax) || cmd.vmax < 0.0f) cmd.vmax = 0.0f;
    if (cmd.vmax > APP_CHASSIS_VMAX_LIMIT) cmd.vmax = APP_CHASSIS_VMAX_LIMIT;
    float mixed[4] = {0};
    if (mode == CHASSIS_DRIVE) wheel_mix(&cmd, mixed);
    float velocity[4];
    for (uint8_t i = 0u; i < 4u; i++) {
        if (APP_CHASSIS_TROT_DRIVE && mode == CHASSIS_DRIVE && legs_mode == LEGS_TROT)
            mixed[i] *= wheel_phase_scale(app_legs_trot_phase(i), APP_CHASSIS_BLEND);
        s_speed[i] = mode == CHASSIS_OFF ? 0.0f :
            wheel_ramp(s_speed[i], mixed[i], APP_CHASSIS_ACC, APP_CHASSIS_DEC,
                       APP_CHASSIS_PERIOD_MS * 0.001f);
        g_chassis.target[i] = s_speed[i];
        velocity[i] = s_speed[i] * s_dir[i];
    }
    /* 浮点计算在区外；只成组发布驱动输入，不调用主循环状态机。 */
    key = bsp_critical_enter();
    for (uint8_t i = 0u; i < 4u; i++) {
        g_motorQZB[i].enable = mode != CHASSIS_OFF;
        if (mode == CHASSIS_OFF) continue;
        g_motorQZB[i].mode = MOTORQZB_MODE_SPEED;
        g_motorQZB[i].vel_kd = mode == CHASSIS_HOLD ? APP_CHASSIS_HOLD_KD : APP_CHASSIS_DRIVE_KD;
        g_motorQZB[i].vel = velocity[i];
    }
    s_owned = 1u;
    bsp_critical_leave(key);
    log_snapshot();
}
