/*
 * 本文件属于 App 应用层，定义四足整机的被动、保持、站立和对角小跑控制接口。
 * 电机接口为 app_motorSHS.h 的 g_motorSHS[]，姿态反馈为 app_imu.h 的 g_imu；依赖 <stdint.h> 与 app_log.h。
 * app_ctrl 依次调用 IMU、整机、电机和记录器任务，每拍 2 ms。
 * 腿下标：0 LF 左前、1 RF 右前、2 LB 左后、3 RB 右后；关节下标：0 髋、1 膝。
 * 机体系以各腿髋部为原点，+x 向前、+z 向下；四条腿的腿系 x 均与机体系 x 同向（左右腿机械镜像）。
 * q 使用腿系 rad；p、p_ref 使用机体系 m；f 使用腿系 N，表示足端对地施力；tau 使用关节侧 Nm。
 * 电机反馈 pos、vel 分别为转子圈数、转子圈数/s；力矩按 LEGS_TORQUE_RATIO 换算为电调指令。
 * 调试步骤：
 * 1. 上电保持折叠姿态，cmd=PASSIVE；等待 8 个电机在线后写 zero_req=1，确认 zeroed=1、zero_req=0。
 * 2. 保持 PASSIVE，手动掰腿，观察 g_legs.q 和 g_legs.p，确认足端位置、左右镜像与实际一致。
 * 3. 调低 hold_kp、hold_kd 和 tau_max，写 cmd=HOLD，低增益验证各关节力矩方向及电调力矩单位。
 *    空载/吊腿阶段先把 tau_max 写到 6 以下，落地站立再恢复默认 15。
 * 4. 写 cmd=STAND，足端以 1.5s 渐变到 stand_x、stand_z；默认 stand_x=-0.02、ff_z=32.6，按需要调整 VMC 增益与 ff_z。
 * 5. 站立渐变结束后写 cmd=TROT，从 stride=0 开始，再逐步调整步长、周期和抬腿高度。
 * 6. IMU：静置时确认 g_imu.ok=1、init_err=0、roll/pitch≈0，抬左侧机身 roll 增大、压机头 pitch 增大；确认后再写 att_kp/ki/kd。
 * 写 cmd=PASSIVE：HOLD/故障/离线/未零位立即撤销使能和力矩；STAND/TROT 且全部电机 RUN 时先缓降到 LEGS_Z_LOWEST 再失能。
 * 降落期间再写非 PASSIVE 命令则取消降落；TROT 中写 cmd=STAND 会从当前足端重新渐变。
 * 故障会退回 PASSIVE；fault 保留最近故障，成功记录零位后清除。零位只能在 PASSIVE 且全在线时记录。
 */
#ifndef APP_LEGS_H
#define APP_LEGS_H

#include <stdint.h>
#include "app_log.h"

#ifndef APP_LEGS_PERIOD_MS
/* 整机控制调度周期，单位毫秒。 */
#define APP_LEGS_PERIOD_MS 2u
#endif

/* 用户指定站高范围 m；变化速率 m/s，默认慢速调节。 */
#ifndef LEGS_Z_MIN
#define LEGS_Z_MIN 0.20f
#endif
#ifndef LEGS_Z_MAX
#define LEGS_Z_MAX 0.38f
#endif
#ifndef LEGS_Z_RATE
#define LEGS_Z_RATE 0.05f
#endif

/* 被动模式前的最低高度 m（至少 LEGS_L_MIN + 0.01 m）及下降速率 m/s。 */
#ifndef LEGS_Z_LOWEST
#define LEGS_Z_LOWEST 0.15f
#endif
#ifndef LEGS_LOWER_RATE
#define LEGS_LOWER_RATE 0.05f
#endif

/* 含义：
 * PASSIVE 为被动模式、HOLD 为保持折叠姿态、STAND 为站立模式、TROT 为对角小跑模式。
*/
typedef enum { LEGS_PASSIVE=0, LEGS_HOLD=1, LEGS_STAND=2, LEGS_TROT=3 } legs_mode_t;

/* 故障类型：
 * NONE 为无故障、NOT_ZEROED 为零位未记录、MOTOR 为电机故障、
 * KIN 为运动学错误、BAD_CMD 为无效命令。
*/
typedef enum {
    LEGS_FAULT_NONE=0, LEGS_FAULT_NOT_ZEROED, LEGS_FAULT_MOTOR,
    LEGS_FAULT_KIN, LEGS_FAULT_BAD_CMD
} legs_fault_t;

typedef struct {  /* 调试接口：输入由调试器写，输出由 app_legs 写 */
    volatile uint8_t cmd;       /* 输入：请求模式 legs_mode_t */
    volatile uint8_t zero_req;  /* 输入：写 1 = 在 PASSIVE 且 8 个电机在线时把当前姿态记为折叠零位；完成后清 0 */
    volatile float stand_x, stand_z;       /* 站立足端目标位置 m（机体系，+x 前，+z 下），默认 -0.02、0.28；质心待实测确认 */
    volatile float kp_x, kp_z, kd_x, kd_z; /* VMC 刚度 N/m、阻尼 N·s/m，默认 600、1200、20、30 */
    volatile float ff_z;                   /* 站立时每腿重力前馈 N，默认 32.6（按加轮整机约 13.3 kg 估算 m·g/4，称重后修正）；TROT 支撑相自动按两腿支撑加倍 */
    volatile float hold_kp, hold_kd;        /* HOLD 关节 PD，Nm/rad、Nm·s/rad，默认 15、0.3 */
    volatile float tau_max;                /* 关节力矩限幅 Nm，默认 15；非负。空载、吊腿调试先写到 6 以下 */
    volatile float period, stride, height; /* 步态周期 s（默认 0.5，下限 0.2）、步长 m（默认 0）、抬腿高度 m（默认 0.05） */
    volatile float att_kp, att_ki, att_kd; /* 姿态修正增益：无量纲、1/s、s；默认 0（关闭），需 g_imu.ok=1；仿真推荐 kp=1.5、ki=1、kd=0.1 */
    volatile float att_dz_max;             /* 姿态修正每腿 z 限幅 m，默认 0.04 */
    volatile float dz[4];                  /* 输出：每腿姿态修正量 m（+ 为伸腿） */
    volatile uint8_t mode;      /* 输出：当前模式 legs_mode_t */
    volatile uint8_t zeroed;    /* 输出：1 = 零位已记录 */
    volatile uint8_t fault;     /* 输出：legs_fault_t，最近一次故障 */
    volatile float q[4][2], p[4][2], p_ref[4][2], f[4][2], tau[4][2];
    /* 输出：每腿关节角 rad、足端 m（机体系）、足端目标 m（机体系）、虚拟力 N（腿系）、关节力矩 Nm */
} legs_debug_t;

extern volatile legs_debug_t g_legs;
extern const sim_vars_t g_legs_vars;

/* app_legs_init：清空整机状态并设置调试参数默认值
 * 说明：由 app_ctrl 在定时器启动前调用一次，默认保持 PASSIVE。
 */
void app_legs_init(void);
/* app_legs_tick：采样电机反馈、处理模式请求并计算整机力矩目标
 * 说明：每 APP_LEGS_PERIOD_MS 调用；控制节拍毫秒差换算为秒，并限幅到 [0.001, 0.02]。
 */
void app_legs_tick(void);

/* app_legs_trot_phase：只读 LF/RF/LB/RB 当前相位 [0,1)，非 TROT 返回支撑相位 0.75。 */
float app_legs_trot_phase(uint8_t leg);

#endif
