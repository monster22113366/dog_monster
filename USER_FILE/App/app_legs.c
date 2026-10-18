/*
 * 本文件属于 App 应用层，组合 PID、腿运动学、VMC 和步态组件实现四足整机控制。
 * 仅通过 g_motorSHS[] 读取转子位置、速度和电机阶段，并写入力矩模式、使能和目标。
 * 通过 app_imu.h 的 g_imu 读取机身姿态，站立渐变结束后及小跑中按增益修正足端高度。
 * 调试参数及观测量集中在 g_legs；其余跨拍状态集中在 s_legs，组件不访问硬件。
 * 依赖 app_ctrl 控制时基及 app_log 描述符接口。
 */
#include "app_legs.h"
#include "app_motorSHS.h"
#include "app_imu.h"
#include "pid.h"
#include "leg_kin.h"
#include "vmc.h"
#include "gait.h"
#include "app_ctrl.h"
#include <math.h>
#include <string.h>

#ifndef LEGS_L1
/* 大腿杆长，单位 m。 */
#define LEGS_L1 0.210f
#endif
#ifndef LEGS_L2
/* 小腿杆长，单位 m。 */
#define LEGS_L2 0.250f
#endif
#ifndef LEGS_L_MIN
/* 上一代工程膝角 33°~143° 对应腿长范围的下限，单位 m。 */
#define LEGS_L_MIN 0.136f
#endif
#ifndef LEGS_L_MAX
/* 上一代工程膝角 33°~143° 对应腿长范围的上限，单位 m。 */
#define LEGS_L_MAX 0.436f
#endif
#ifndef LEGS_GEAR
/* 转子圈数 / 关节圈数。 */
#define LEGS_GEAR 8.0f
#endif
#ifndef LEGS_TORQUE_RATIO
/* 关节（输出轴）力矩 / 电调力矩指令，取减速比 8。
 * 依据：控制算法参考工程 Corgi 在算法层按输出轴力矩计算，下发时除以减速比换算为转子侧
 * （A1_motor.c:89 target.T / reduction_ratio）；本协议 0x009 位置、速度同为转子侧。 */
#define LEGS_TORQUE_RATIO 8.0f
#endif
#ifndef LEGS_Q_FOLD_HIP
/* 折叠零位髋角，单位 rad；上一代两点标定换算得到 -55.44°。 */
#define LEGS_Q_FOLD_HIP -0.96761f
#endif
#ifndef LEGS_Q_FOLD_KNEE
/* 折叠零位膝角，单位 rad；上一代两点标定换算得到 124.47°，足端约 (0.077, 0.112) m。 */
#define LEGS_Q_FOLD_KNEE 2.17241f
#endif
#ifndef LEGS_STAND_RAMP_S
/* 站立足端渐变时长，单位 s。 */
#define LEGS_STAND_RAMP_S 1.5f
#endif
#ifndef LEGS_STRIDE_RATE
/* 步长变化速率上限，单位 m/s。 */
#define LEGS_STRIDE_RATE 0.1f
#endif
#ifndef LEGS_F_MAX
/* 每条腿各轴虚拟力绝对值上限，单位 N。 */
#define LEGS_F_MAX 200.0f
#endif
#ifndef LEGS_GAIT_APEX
/* 摆动段内抬腿最高点的位置比例，无量纲。 */
#define LEGS_GAIT_APEX 0.32f
#endif
#ifndef LEGS_HIP_X
/* 髋关节轴（大腿转轴、膝电机所在轴）到机身中心的前后距离，单位 m。
 * body.step 电机板 z=±200 mm 孔，前后髋轴间距 40 cm；用户量得的髋电机出轴间距
 * 20 cm 是同步带主动轴间距，不是该值；待实物确认。 */
#define LEGS_HIP_X 0.200f
#endif
#ifndef LEGS_HIP_Y
/* 腿平面到机身中心的横向距离，单位 m，用户实测。 */
#define LEGS_HIP_Y 0.142f
#endif
#ifndef LEGS_FF_BLEND
/* TROT 支撑/摆动切换处重力前馈的过渡宽度，占步态周期的比例，无量纲。 */
#define LEGS_FF_BLEND 0.1f
#endif

/* 按 |stand_x| <= 0.05 m 校验；距最大腿长另留 0.01 m 余量。 */
__extension__ _Static_assert(LEGS_Z_MIN >= LEGS_L_MIN && LEGS_Z_MIN <= LEGS_Z_MAX &&
    0.05f * 0.05f + LEGS_Z_MAX * LEGS_Z_MAX <=
    (LEGS_L_MAX - 0.01f) * (LEGS_L_MAX - 0.01f), "stand height exceeds leg reach");

_Static_assert(APP_MOTORSHS_NUM == 8u, "legs requires eight motors");
__extension__ _Static_assert(LEGS_Z_LOWEST >= LEGS_L_MIN + 0.01f,
    "lowering height below minimum leg reach");

/* 参数含义:
 * hip: 髋关节
 * knee: 膝关节
 * s_hip/s_knee: sign_hip/sign_knee,髋/膝关节方向
 * x_sign: x 轴方向。

 * 腿与电机映射：
 * (app_motorSHS.c)里的电机数组 motors[] 下标：0~3 = CAN1 节点 1~4（前腿），4~7 = CAN2 节点 1~4（后腿）；
 * 节点 2/1 = 左腿 髋/膝，节点 4/3 = 右腿 髋/膝（与上一代一致）。
 * 安装：以狗为参照，节点 1、2 在左侧且输出轴朝左，节点 3、4 在右侧且输出轴朝右。

 * 推导过程：
 * 实测：力矩模式 target 为正时，正对输出轴看电机顺时针转；且 fb.pos 增大、fb.vel 为正，
 * 即编码器正方向与力矩正方向一致，关节角与力矩共用 s_hip/s_knee。
 * 左右腿机械镜像，四条腿的腿系都与机体系 x 同向（+x 前，+z 下），折叠时足端都在髋前。
 * 推导：左侧顺时针 = 足端向前 = 腿系 q 增大，符号 +1；
 * 右侧顺时针 = 足端向后 = 腿系 q 减小，符号 -1。
 * x_sign：四条腿均为 +1（腿系 x = 机体 x * x_sign），保留该字段便于以后适配其他构型。 */
static const struct {
    uint8_t hip, knee;
    int8_t s_hip, s_knee, x_sign;
} legs[4] = {
    {1, 0, +1, +1, +1}, /* LF：CAN1 髋=节点2，膝=节点1 */
    {3, 2, -1, -1, +1}, /* RF：CAN1 髋=节点4，膝=节点3 */
    {5, 4, +1, +1, +1}, /* LB：CAN2 髋=节点2，膝=节点1 */
    {7, 6, -1, -1, +1}, /* RB：CAN2 髋=节点4，膝=节点3 */
};

/* 腿杆长度。 */
static const leg_geom_t geometry = {LEGS_L1, LEGS_L2, LEGS_L_MIN, LEGS_L_MAX};
/* Live Watch 唯一公开的整机调试状态。 */
volatile legs_debug_t g_legs;

/* 二维量按行主序展平，足端位置使用机体系；记录器只读取普通存储快照。 */
static struct {
    float p[8], p_ref[8], tau[8], dz[4];
    uint8_t mode, zeroed, fault;
} s_legs_log;
static const sim_var_desc_t s_legs_vars[] = {
    {"p", SIM_VAR_F32, 8u, s_legs_log.p},
    {"p_ref", SIM_VAR_F32, 8u, s_legs_log.p_ref},
    {"tau", SIM_VAR_F32, 8u, s_legs_log.tau},
    {"mode", SIM_VAR_U8, 0u, &s_legs_log.mode},
    {"zeroed", SIM_VAR_U8, 0u, &s_legs_log.zeroed},
    {"fault", SIM_VAR_U8, 0u, &s_legs_log.fault},
    {"dz", SIM_VAR_F32, 4u, s_legs_log.dz},
};
const sim_vars_t g_legs_vars = {"legs", s_legs_vars, sizeof(s_legs_vars) / sizeof(s_legs_vars[0])};

/* log_snapshot：发布本拍实际足端、足端目标、关节力矩、姿态修正与整机状态。 */
static void log_snapshot(void) {
    for (uint8_t i = 0u; i < 4u; i++) {
        for (uint8_t j = 0u; j < 2u; j++) {
            s_legs_log.p[2u * i + j] = g_legs.p[i][j];
            s_legs_log.p_ref[2u * i + j] = g_legs.p_ref[i][j];
            s_legs_log.tau[2u * i + j] = g_legs.tau[i][j];
        }
        s_legs_log.dz[i] = g_legs.dz[i];
    }
    s_legs_log.mode = g_legs.mode;
    s_legs_log.zeroed = g_legs.zeroed;
    s_legs_log.fault = g_legs.fault;
}

/* 所有可变跨拍状态集中保存，角度为 rad，位置为 m，时间为 s（last_ms 除外）。 */
static struct {
    float pos0[8];             /* 折叠零位的转子圈数。 */
    uint32_t last_ms;          /* 上一次调用的控制节拍 ms。 */
    legs_mode_t mode;
    uint8_t entry_pending;     /* 等待全部电机 RUN 后再捕获入口姿态。 */
    uint8_t lowering;          /* PASSIVE 请求下先站立缓降，再撤销使能。 */
    float q_ref[4][2];
    pid_ctrl_t pid[4][2];
    float ramp_start[4][2];    /* 站立渐变起点，使用腿系。 */
    float ramp_time;
    float trot_clock;         /* 归一化周期相位，[0,1)。 */
    float stride_cur;
    float z_cur;               /* 限幅限速后的名义高度，仅 STAND 推进。 */
    float last_p_ref[4][2];    /* 上一拍腿系目标，用于数值求导。 */
    uint8_t first_tick;
    float att_i[2];            /* 姿态积分 {roll, pitch}，rad·s。 */
} s_legs;

/* app_legs_trot_phase：复用整机小跑时钟，LF/RB 同相，RF/LB 相差半周期。 */
float app_legs_trot_phase(uint8_t leg) {
    if (g_legs.mode != LEGS_TROT || leg >= 4u)
        return 0.75f;
    float phase = s_legs.trot_clock + ((leg == 0u || leg == 3u) ? 0.0f : 0.5f);
    return phase - floorf(phase);
}

/* 单拍腿系反馈，不保存跨拍历史。 
 * q[2] = {qh, qk}，是关节角
 * p[2] = {x, z}，是足端位置
 * J[2][2] = {dx/dqh, dx/dqk; dz/dqh, dz/dqk}，是雅可比矩阵
 * v[2] = {dx/dt, dz/dt}，是足端速度
*/

typedef struct {
    float q[2], p[2], J[2][2], v[2];
} leg_feedback_t;

/* update_feedback：把转子反馈的转子位置，电机速度换算为关节角，再把关节角换算为足端位置、足端速度和雅可比矩阵(足端位置 {x, z} 对两个电机角 {qh, qk} 的偏导数)
 * 参数：fb 为同一拍电机快照；state 接收四腿运动学结果
 * 返回：任一腿运动学失败或出现非有限反馈时为 1，否则为 0。
 */
static uint8_t update_feedback(const mwSHS_motor_t fb[8], leg_feedback_t state[4]) {
    const float angle_scale = 6.2831853071795864769f / LEGS_GEAR;
    uint8_t kin_error = 0u;
    for (uint8_t i = 0u; i < 4u; i++) {
        const uint8_t h = legs[i].hip, k = legs[i].knee;
        leg_feedback_t *leg = &state[i];
        const float qd[2] = {legs[i].s_hip * fb[h].vel * angle_scale,
                             legs[i].s_knee * fb[k].vel * angle_scale};
        leg->q[0] = legs[i].s_hip * (fb[h].pos - s_legs.pos0[h]) * angle_scale + LEGS_Q_FOLD_HIP;
        leg->q[1] = legs[i].s_knee * (fb[k].pos - s_legs.pos0[k]) * angle_scale + LEGS_Q_FOLD_KNEE;
        g_legs.q[i][0] = leg->q[0];
        g_legs.q[i][1] = leg->q[1];
        if (!isfinite(leg->q[0]) || !isfinite(leg->q[1]) ||
            !isfinite(qd[0]) || !isfinite(qd[1]) ||
            leg_fk(&geometry, leg->q[0], leg->q[1], leg->p) != 0 ||
            !isfinite(leg->p[0]) || !isfinite(leg->p[1])) {
            kin_error = 1u;
            continue;
        }
        leg_jacobian(&geometry, leg->q[0], leg->q[1], leg->J);
        vmc_foot_vel((const float (*)[2])leg->J, qd, leg->v);
        if (!isfinite(leg->v[0]) || !isfinite(leg->v[1]))
            kin_error = 1u;
        g_legs.p[i][0] = leg->p[0] * legs[i].x_sign;
        g_legs.p[i][1] = leg->p[1];
    }
    return kin_error;
}

/* clamp_tau：关节力矩对称限幅，NaN 输出按零处理；limit 已保证非负。 */
static float clamp_tau(float tau, float limit) {
    if (isnan(tau))
        return 0.0f;
    return tau < -limit ? -limit : (tau > limit ? limit : tau);
}

/* stance_weight：步态相位对应的支撑权重，[0.5,1) 为支撑段，在 0.5 和 1.0 两个边界做对称的 s5 过渡。
 * 对角两组相位差 0.5，两组权重之和恒为 1，因此总支撑力连续。 */
static float stance_weight(float phase) {
    const float h = 0.5f * LEGS_FF_BLEND;
    if (phase < 0.5f - h)
        phase += 1.0f;
    return gait_s5((phase - (0.5f - h)) / LEGS_FF_BLEND) * gait_s5(((1.0f + h) - phase) / LEGS_FF_BLEND);
}

/* att_correct：由机身 roll/pitch 计算各腿足端 z 目标修正量（+z 向下 = 伸腿 = 抬高该角）
 * 参数：on 为 1 时计算并积分，0 时输出 0 且清积分；dt 为本拍间隔 s；dz 接收四腿修正 m
 * 返回：无
 * 说明：角点高度偏差 ≈ y·roll - x·pitch，修正 dz = -(y·u_roll - x·u_pitch)，u = kp·角 + ki·∫角 + kd·角速度；
 *       每腿限幅 ±att_dz_max，积分按同一限幅抗饱和。增益全为 0 时等价于关闭。
 */
static void att_correct(uint8_t on, float dt, float dz[4]) {
    static const float hx[4] = {LEGS_HIP_X, LEGS_HIP_X, -LEGS_HIP_X, -LEGS_HIP_X};
    static const float hy[4] = {LEGS_HIP_Y, -LEGS_HIP_Y, LEGS_HIP_Y, -LEGS_HIP_Y};
    const float lim = g_legs.att_dz_max > 0.0f ? g_legs.att_dz_max : 0.0f;
    if (!on || !g_imu.ok) {
        s_legs.att_i[0] = s_legs.att_i[1] = 0.0f;
        for (uint8_t i = 0u; i < 4u; i++)
            dz[i] = 0.0f;
        return;
    }
    const float ang[2] = {g_imu.roll, g_imu.pitch};
    const float rate[2] = {g_imu.gyro[0], g_imu.gyro[1]};
    const float arm[2] = {LEGS_HIP_Y, LEGS_HIP_X};
    float u[2];
    for (uint8_t k = 0u; k < 2u; k++) {
        s_legs.att_i[k] += ang[k] * dt;
        if (g_legs.att_ki > 0.0f) {
            const float i_max = lim / (arm[k] * g_legs.att_ki);
            s_legs.att_i[k] = s_legs.att_i[k] < -i_max ? -i_max : (s_legs.att_i[k] > i_max ? i_max : s_legs.att_i[k]);
        } else {
            s_legs.att_i[k] = 0.0f;
        }
        u[k] = g_legs.att_kp * ang[k] + g_legs.att_ki * s_legs.att_i[k] + g_legs.att_kd * rate[k];
    }
    for (uint8_t i = 0u; i < 4u; i++) {
        const float d = -(hy[i] * u[0] - hx[i] * u[1]);
        dz[i] = d < -lim ? -lim : (d > lim ? lim : d);
    }
}

/* app_legs_init：复位整机内部状态与调试接口，设置默认控制参数
 * 参数：无
 * 返回：无
 * 说明：姿态增益由清零操作设为 0，默认关闭修正。
 */
void app_legs_init(void) {
    memset((void *)&g_legs, 0, sizeof(g_legs));
    memset(&s_legs, 0, sizeof(s_legs));
    /* 仿真显示后移 20 mm 可降低姿态/膝关节峰值；质心待实测确认。 */
    g_legs.stand_x = -0.02f;
    g_legs.stand_z = 0.28f;
    s_legs.z_cur = g_legs.stand_z;
    g_legs.kp_x = 600.0f;
    g_legs.kp_z = 1200.0f;
    g_legs.kd_x = 20.0f;
    g_legs.kd_z = 30.0f;
    /* 按加轮整机约 13.3 kg 估算 m·g/4；称重后修正。 */
    g_legs.ff_z = 32.6f;
    g_legs.hold_kp = 15.0f;
    g_legs.hold_kd = 0.3f;
    g_legs.tau_max = 15.0f;
    g_legs.period = 0.5f;
    g_legs.stride = 0.0f;
    g_legs.height = 0.05f;
    g_legs.att_dz_max = 0.04f;
    /* 控制时基在拍末推进；首拍同样按完整的 2 ms 积分。 */
    s_legs.last_ms = app_ctrl_ms() - APP_CTRL_PERIOD_MS;
    log_snapshot();
}

/* app_legs_tick：处理反馈、零位和模式切换，最后统一发布每腿与每电机目标
 * 参数：无
 * 返回：无
 * 说明：非 PASSIVE 模式仅在全部电机 RUN 且入口初始化完成后输出力矩。
 * 离线、电机故障和运动学错误立即退回 PASSIVE；毫秒计时差支持自然回绕。
 */
void app_legs_tick(void) {
    const uint32_t now = app_ctrl_ms();
    float dt = (uint32_t)(now - s_legs.last_ms) * 0.001f;
    s_legs.last_ms = now;
    dt = dt < 0.001f ? 0.001f : (dt > 0.02f ? 0.02f : dt);

    mwSHS_motor_t fb[8];
    leg_feedback_t state[4] = {0};
    uint8_t online = 1u, running = 1u, motor_fault = 0u;
    for (uint8_t i = 0u; i < 8u; i++) {
        memcpy(&fb[i], (const void *)&g_motorSHS[i].fb, sizeof(fb[i]));
        const app_motorSHS_phase_t phase = g_motorSHS[i].phase;
        if (phase == APP_SHS_PH_OFFLINE)
            online = 0u;
        if (phase != APP_SHS_PH_RUN)
            running = 0u;
        if (phase == APP_SHS_PH_FAULT)
            motor_fault = 1u;
    }

    uint8_t kin_error = g_legs.zeroed ? update_feedback(fb, state) : 0u;
    if (g_legs.zero_req && s_legs.mode == LEGS_PASSIVE && online) {
        for (uint8_t i = 0u; i < 8u; i++)
            s_legs.pos0[i] = fb[i].pos;
        g_legs.zeroed = 1u;
        g_legs.zero_req = 0u;
        g_legs.fault = LEGS_FAULT_NONE;
        /* 零位与模式请求可在同一拍到达，立即重算，避免入口捕获旧坐标。 */
        kin_error = update_feedback(fb, state);
    }

    uint8_t requested = g_legs.cmd;
    if (requested > LEGS_TROT) {
        g_legs.fault = LEGS_FAULT_BAD_CMD;
        requested = (uint8_t)s_legs.mode;
        g_legs.cmd = requested;
    }
    if (requested != LEGS_PASSIVE && !g_legs.zeroed) {
        g_legs.fault = LEGS_FAULT_NOT_ZEROED;
        requested = LEGS_PASSIVE;
        g_legs.cmd = requested;
    }
    if (requested == LEGS_TROT && s_legs.mode != LEGS_TROT &&
        (s_legs.mode != LEGS_STAND || s_legs.entry_pending || s_legs.ramp_time < LEGS_STAND_RAMP_S)) {
        requested = (uint8_t)s_legs.mode;
        g_legs.cmd = requested;
    }
    if (requested != LEGS_PASSIVE) {
        s_legs.lowering = 0u;
    } else if (g_legs.zeroed && running && !motor_fault && online && !kin_error &&
               (s_legs.mode == LEGS_STAND || s_legs.mode == LEGS_TROT)) {
        s_legs.lowering = 1u;
    }
    if (s_legs.lowering && g_legs.cmd == LEGS_PASSIVE && g_legs.zeroed)
        requested = LEGS_STAND;
    if (requested != s_legs.mode) {
        s_legs.mode = (legs_mode_t)requested;
        s_legs.entry_pending = 1u;
        if (s_legs.mode == LEGS_PASSIVE)
            s_legs.lowering = 0u;
    }
    if (s_legs.mode != LEGS_PASSIVE && (motor_fault || !online || kin_error)) {
        s_legs.mode = LEGS_PASSIVE;
        s_legs.entry_pending = 1u;
        s_legs.lowering = 0u;
        g_legs.cmd = LEGS_PASSIVE;
        g_legs.fault = motor_fault || !online ? LEGS_FAULT_MOTOR : LEGS_FAULT_KIN;
    }

    float tau_max = g_legs.tau_max;
    if (!(tau_max >= 0.0f))
        tau_max = 0.0f;
    if (s_legs.entry_pending && (running || s_legs.mode == LEGS_PASSIVE)) {
        if (s_legs.mode == LEGS_HOLD) {
            for (uint8_t i = 0u; i < 4u; i++) {
                for (uint8_t j = 0u; j < 2u; j++) {
                    s_legs.q_ref[i][j] = state[i].q[j];
                    pid_init(&s_legs.pid[i][j], g_legs.hold_kp, 0.0f, g_legs.hold_kd, 0.0f, tau_max);
                }
            }
        } else if (s_legs.mode == LEGS_STAND) {
            for (uint8_t i = 0u; i < 4u; i++)
                memcpy(s_legs.ramp_start[i], state[i].p, sizeof(state[i].p));
            s_legs.ramp_time = 0.0f;
        } else if (s_legs.mode == LEGS_TROT) {
            s_legs.trot_clock = 0.0f;
            s_legs.stride_cur = 0.0f;
            s_legs.first_tick = 1u;
        }
        s_legs.entry_pending = 0u;
    }

    if (s_legs.mode == LEGS_STAND)
        s_legs.z_cur = gait_slew(s_legs.z_cur,
                                 s_legs.lowering ? LEGS_Z_LOWEST : g_legs.stand_z,
                                 (s_legs.lowering ? LEGS_LOWER_RATE : LEGS_Z_RATE) * dt,
                                 s_legs.lowering ? LEGS_Z_LOWEST : LEGS_Z_MIN, LEGS_Z_MAX);

    if (s_legs.lowering && !s_legs.entry_pending &&
        s_legs.ramp_time >= LEGS_STAND_RAMP_S && s_legs.z_cur <= LEGS_Z_LOWEST + 0.002f) {
        s_legs.mode = LEGS_PASSIVE;
        s_legs.entry_pending = 1u;
        s_legs.lowering = 0u;
    }

    const uint8_t active = s_legs.mode != LEGS_PASSIVE && running && !s_legs.entry_pending;
    /* 姿态修正只在站立渐变结束后及小跑中生效；其余时刻清积分。 */
    float dz[4];
    att_correct(active && (s_legs.mode == LEGS_TROT ||
                           (s_legs.mode == LEGS_STAND && s_legs.ramp_time >= LEGS_STAND_RAMP_S)), dt, dz);
    for (uint8_t i = 0u; i < 8u; i++) {
        if (s_legs.mode != LEGS_PASSIVE)
            g_motorSHS[i].mode = APP_SHS_MODE_TORQUE;
        g_motorSHS[i].enable = s_legs.mode != LEGS_PASSIVE;
        g_motorSHS[i].target = 0.0f;
    }
    if (active && s_legs.mode == LEGS_TROT) {
        const float step = LEGS_STRIDE_RATE * dt;
        const float delta = g_legs.stride - s_legs.stride_cur;
        s_legs.stride_cur += delta < -step ? -step : (delta > step ? step : delta);
        const float period = g_legs.period;
        s_legs.trot_clock += dt / (period < 0.2f ? 0.2f : period);
        s_legs.trot_clock -= floorf(s_legs.trot_clock);
    }

    for (uint8_t i = 0u; i < 4u; i++) {
        const leg_feedback_t *leg = &state[i];
        float p_ref[2] = {leg->p[0], leg->p[1]};
        float v_ref[2] = {0.0f, 0.0f};
        float f[2] = {0.0f, 0.0f}, tau[2] = {0.0f, 0.0f};
        float ff_z = g_legs.ff_z;
        if (active && s_legs.mode == LEGS_HOLD) {
            for (uint8_t j = 0u; j < 2u; j++) {
                pid_ctrl_t *pid = &s_legs.pid[i][j];
                pid->kp = g_legs.hold_kp;
                pid->kd = g_legs.hold_kd;
                pid->out_max = tau_max;
                tau[j] = pid_calc(pid, s_legs.q_ref[i][j], leg->q[j], dt);
            }
            (void)leg_fk(&geometry, s_legs.q_ref[i][0], s_legs.q_ref[i][1], p_ref);
        } else if (active && s_legs.mode == LEGS_STAND) {
            const float target[2] = {g_legs.stand_x * legs[i].x_sign, s_legs.z_cur};
            const float s = gait_s5(s_legs.ramp_time / LEGS_STAND_RAMP_S);
            for (uint8_t j = 0u; j < 2u; j++)
                p_ref[j] = s_legs.ramp_start[i][j] + (target[j] - s_legs.ramp_start[i][j]) * s;
        } else if (active && s_legs.mode == LEGS_TROT) {
            float phase = s_legs.trot_clock + ((i == 0u || i == 3u) ? 0.0f : 0.5f);
            phase -= floorf(phase);
            const gait_param_t gait = {s_legs.stride_cur, g_legs.height, s_legs.z_cur, LEGS_GAIT_APEX};
            gait_trot_foot(&gait, phase, p_ref);
            p_ref[0] = (p_ref[0] + g_legs.stand_x) * legs[i].x_sign;
            for (uint8_t j = 0u; j < 2u; j++) {
                if (!s_legs.first_tick)
                    v_ref[j] = (p_ref[j] - s_legs.last_p_ref[i][j]) / dt;
                s_legs.last_p_ref[i][j] = p_ref[j];
            }
            ff_z = 2.0f * g_legs.ff_z * stance_weight(phase);
        }
        g_legs.dz[i] = dz[i];
        if (active && (s_legs.mode == LEGS_STAND || s_legs.mode == LEGS_TROT)) {
            p_ref[1] += dz[i];
            const vmc_gain_t gains = {{g_legs.kp_x, g_legs.kp_z},
                                     {g_legs.kd_x, g_legs.kd_z},
                                     {0.0f, ff_z}, {LEGS_F_MAX, LEGS_F_MAX}};
            vmc_force(&gains, p_ref, v_ref, leg->p, leg->v, f);
            vmc_torque(leg->J, f, tau);
        }
        for (uint8_t j = 0u; j < 2u; j++) {
            tau[j] = clamp_tau(tau[j], tau_max);
            g_legs.tau[i][j] = tau[j];
            g_legs.f[i][j] = f[j];
            g_legs.p_ref[i][j] = j == 0u ? p_ref[j] * legs[i].x_sign : p_ref[j];
        }
        if (active) {
            g_motorSHS[legs[i].hip].target = legs[i].s_hip * tau[0] / LEGS_TORQUE_RATIO;
            g_motorSHS[legs[i].knee].target = legs[i].s_knee * tau[1] / LEGS_TORQUE_RATIO;
        }
    }
    if (active && s_legs.mode == LEGS_STAND) {
        s_legs.ramp_time += dt;
        if (s_legs.ramp_time > LEGS_STAND_RAMP_S)
            s_legs.ramp_time = LEGS_STAND_RAMP_S;
    }
    if (active && s_legs.mode == LEGS_TROT)
        s_legs.first_tick = 0u;
    g_legs.mode = (uint8_t)s_legs.mode;
    log_snapshot();
}
