/* 轮底盘：TIM6 每 2 ms 写入 MIT 速度模式命令，主循环继续推进电机状态机。 */
/*
 * 使用说明（实机调试）：
 * 变量方向：g_chassis.mode_req 为输入，请求 0 OFF/1 HOLD/2 DRIVE，上电默认 1；
 * g_chassis.mode 为只读输出，表示每拍由固件计算的实际生效模式，调试器不要写；
 * g_chassis.enable 为输入，总接管开关默认 0；接管后 1→0 时将四轮 g_motorQZB[i].enable 清零一次（释放），之后不再写。
 * enable 从未置 1 时完全不写 g_motorQZB，台架可直接调 g_motorQZB。
 * 实际模式可能与请求不同：首次四轮未全部在线前保持 OFF；腿为 PASSIVE 时强制 OFF（APP_CHASSIS_IGNORE_LEGS=1 时不生效）；
 * 腿为 TROT 且 APP_CHASSIS_TROT_DRIVE==0 时请求 DRIVE 按 HOLD；轮子曾全部在线后出现 FAULT/OFFLINE 则锁存 OFF。
 * 阶段一（台架空载，只接轮电机，g_chassis.enable 保持 0）：对每个 g_motorQZB[i] 先写 enable=0，
 * 等 phase=1（IDLE），再设 mode（1 力矩/2 速度/3 位置）、vel、vel_kd，最后 enable=1，等 phase=4（RUN）；
 * 停止时先写 vel=0，再写 enable=0。此时若把 g_chassis.enable 置 1，底盘每 2 ms 覆盖 enable/mode/vel/vel_kd。
 * 台架单测底盘（轮子悬空、腿未装）：把 APP_CHASSIS_IGNORE_LEGS 改为 1 重新编译烧录；
 * 四个轮电机仍须全部在线；其余步骤同阶段二；测完务必改回 0。
 * 阶段二（装机、腿已零位校准、8 个腿电机在线、已 STAND）：fwd=yaw=0，mode_req=0，enable=1；
 * 确认 online_mask==0x0F 且 mode 脱离 0，再写 mode_req=1（HOLD，零速阻尼保持），随后 mode_req=2（DRIVE），
 * fwd 从 0.1 起小幅增加。停车写 mode_req=1（按 DEC 减速到零保持），再写 mode_req=0 放开轮子；紧急时写 g_motorSHS_kill=1。
 * 遥控使能期间 app_rc 每 10 ms 把 g_chassis.enable 置 1，调试器写 0 会被覆盖。
 * 想停请先让 SC 回中档、摇杆回中，再把 g_rc.enable 写 0；若遥控接管过底盘，关闭沿
 * 一次性恢复 fwd=yaw=0、mode_req=HOLD、enable=0，底盘随后释放四轮。
 * g_rc.enable=0 默认只监视，除上述关闭沿释放外不写控制量，保留手动台架输入。
 * 遥控默认不使能（APP_RC_ENABLE_DEFAULT=0），要用遥控测底盘需先使能 g_rc.enable（见 app_rc.h）。
 * 方向符号：g_chassis.vel[] 已乘 s_dir，车体前进时四个值都应为正；为负的轮对应的 s_dir[i] 取反。
 * 故障恢复：g_chassis.fault 非 0 表示已锁存 OFF；先写 mode_req=0 并至少保持一拍，再写 1 或 2。
 * 电机本身有故障时，还要对 g_motorQZB[i] 先写 enable=0、clear_error=1。
 * 重点观察：g_chassis{mode,fault,online_mask,running_mask,target[],vel[]}；
 * g_motorQZB[i]{phase,fault,fb.vel,fb.torque}。
 * 调试器暂停内核超过 1 s 时，电机会因 CAN 超时自行失能，恢复后底盘会锁存故障。
 * 单位与限幅：vmax 默认 APP_CHASSIS_VMAX（7 rad/s），上限 APP_CHASSIS_VMAX_LIMIT；
 * 24.5 V 供电下空载轮速上限约 9 rad/s。
 */
#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H
#include "app_log.h"
#include "app_motorQZB.h"

#ifndef APP_CHASSIS_PERIOD_MS
#define APP_CHASSIS_PERIOD_MS 2u /* 与腿控制同拍。 */
#endif
#ifndef APP_CHASSIS_VMAX
/* 24.5 V 空载约 9 rad/s，取 7 留约 25% 余量；换 48 V 电池后可提高 VMAX。
 * 轮半径 0.0575 m；接近空载上限时可用力矩很小。 */
#define APP_CHASSIS_VMAX 7.0f
#endif
#ifndef APP_CHASSIS_VMAX_LIMIT
/* 24.5 V 空载约 9 rad/s；换 48 V 电池后可提高 VMAX，48 V 空载约 18 rad/s。
 * PA43-10-20.25 输出轴 TN 曲线约 174 rpm，当前减速比无法达到 2 m/s。 */
#define APP_CHASSIS_VMAX_LIMIT 18.0f
#endif
#ifndef APP_CHASSIS_ACC
#define APP_CHASSIS_ACC 20.0f /* rad/s²，限制 kd×速度阶跃的力矩。 */
#endif
#ifndef APP_CHASSIS_DEC
#define APP_CHASSIS_DEC 40.0f /* rad/s²，制动快于加速。 */
#endif
#ifndef APP_CHASSIS_DRIVE_KD
#define APP_CHASSIS_DRIVE_KD 1.0f /* 空载整定：误差约 0.2 rad/s，上升约 24 ms。 */
#endif
#ifndef APP_CHASSIS_HOLD_KD
#define APP_CHASSIS_HOLD_KD 2.0f /* 零速阻尼保持，取驱动软限幅上限。 */
#endif
#ifndef APP_CHASSIS_TROT_DRIVE
#define APP_CHASSIS_TROT_DRIVE 0 /* 阶段二小跑时保持；1 留给相位缩放验证。 */
#endif
#ifndef APP_CHASSIS_IGNORE_LEGS
/* 仅限轮子悬空的台架使用；置 1 仅绕过“腿 PASSIVE → 底盘强制 OFF”联锁。
 * 上整机（腿装机、轮子落地）之前必须改回 0。 */
#define APP_CHASSIS_IGNORE_LEGS 1
#endif
#if APP_CHASSIS_IGNORE_LEGS
#warning "APP_CHASSIS_IGNORE_LEGS=1：台架模式，腿联锁已禁用，仅限轮子悬空使用"
#endif
#ifndef APP_CHASSIS_BLEND
#define APP_CHASSIS_BLEND 0.1f /* 相位边界过渡宽度，占一个周期。 */
#endif

typedef enum { CHASSIS_OFF=0, CHASSIS_HOLD=1, CHASSIS_DRIVE=2 } chassis_mode_t;
typedef struct {
    uint8_t enable, mode_req; /* 输入：总接管开关、请求模式；默认 0、HOLD。 */
    float fwd, yaw, vmax;     /* 输入：归一化前进/转向；vmax 为输出轴轮速 rad/s。 */
    uint8_t mode;             /* 输出：实际生效模式 chassis_mode_t。 */
    float target[4], vel[4];  /* 输出：斜坡目标、反馈轮速 rad/s，LF/RF/LB/RB，前进为正。 */
    uint8_t online_mask, running_mask; /* 输出：在线/运行位图，bit0..3 对应 LF/RF/LB/RB。 */
    app_motorQZB_fault_t fault; /* 输出：底盘锁存故障码。 */
} chassis_debug_t;

extern volatile chassis_debug_t g_chassis;
extern const sim_vars_t g_chassis_vars;
void app_chassis_init(void);
void app_chassis_tick(void);
#endif
