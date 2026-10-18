/* 调试器写 g_motorQZB 输入；同一上下文每 2 ms 调用 tick。
 * 发送 FIFO 满逐拍重试，连续忙超时才报 TX 故障；负返回值立即报 TX 故障。 */
#ifndef APP_MOTORQZB_H
#define APP_MOTORQZB_H
#include "mw_canQZB.h"

#define APP_MOTORQZB_COUNT 4u
#define MOTORQZB_MODE_TORQUE 1u
#define MOTORQZB_MODE_SPEED 2u
#define MOTORQZB_MODE_POSITION 3u
#ifndef APP_MOTORQZB_RX_TIMEOUT_MS
#define APP_MOTORQZB_RX_TIMEOUT_MS 200u
#endif
#ifndef APP_MOTORQZB_STEP_TIMEOUT_MS
#define APP_MOTORQZB_STEP_TIMEOUT_MS 500u
#endif
#ifndef APP_MOTORQZB_RETRY_MS
#define APP_MOTORQZB_RETRY_MS 20u
#endif
#ifndef APP_MOTORQZB_TX_BUSY_MS
/* 发送 FIFO 满（mwQZB_send 返回 0）的容忍时间；连续达到该时间才判为发送故障。 */
#define APP_MOTORQZB_TX_BUSY_MS 100u
#endif
/* 首次上机的软件目标限幅，不改变电机内部 Torque Limit=18 Nm。 */
#ifndef APP_MOTORQZB_TQ_LIMIT
#define APP_MOTORQZB_TQ_LIMIT 3.0f
#endif
/* 轮径 115 mm（直径），2 m/s≈34.8 rad/s；通信量程 ±65 rad/s（约 3.7 m/s）。 */
/* 实测空载转速在约 9.05 rad/s 处出现与 kd、目标速度无关的上限，四台电机一致；
 * 原因待确认（怀疑母线电压限制），上层速度指令需按实际供电确认后设置。 */
#ifndef APP_MOTORQZB_VEL_LIMIT
#define APP_MOTORQZB_VEL_LIMIT 60.0f
#endif
/* 首次上机软限幅，与电机内 Torque Limit=18 Nm 无关；
 * 位置模式输出力矩≈kp×位置误差，所以限 kp/kd。 */
#ifndef APP_MOTORQZB_POS_KP_LIMIT
#define APP_MOTORQZB_POS_KP_LIMIT 5.0f
#endif
#ifndef APP_MOTORQZB_POS_KD_LIMIT
#define APP_MOTORQZB_POS_KD_LIMIT 0.5f
#endif
/* 位置/速度增益独立；位置默认增益空载实测（0.5 rad 阶跃）：
 * 上升约 100 ms，超调约 6%，稳态误差约 0.02 rad；
 * 位置反馈在 ±12.5 rad 饱和，位置模式仅用于单轮台架调试。 */
#ifndef APP_MOTORQZB_POS_KP_DEFAULT
#define APP_MOTORQZB_POS_KP_DEFAULT 5.0f
#endif
#ifndef APP_MOTORQZB_POS_KD_DEFAULT
#define APP_MOTORQZB_POS_KD_DEFAULT 0.2f
#endif
/* 速度模式 MIT 阻尼增益，空载实测折中；稳态速度误差≈负载力矩/kd，
 * 空载 0.2 Nm 时约 0.2 rad/s，5 rad/s 阶跃上升到 90% 约 24 ms；
 * 零速保持的蠕动速度≈外力矩/kd，落地后需复调；
 * 大阶跃请由上层斜坡限制，否则 kd×Δv 会冲到电机力矩上限。 */
#ifndef APP_MOTORQZB_VEL_KD_DEFAULT
#define APP_MOTORQZB_VEL_KD_DEFAULT 1.0f
#endif
/* 限制速度误差产生的力矩 kd×Δv，首次上机软限幅，待实机整定；
 * 电机内 Torque Limit=18 Nm 另有保护。 */
#ifndef APP_MOTORQZB_VEL_KD_LIMIT
#define APP_MOTORQZB_VEL_KD_LIMIT 2.0f
#endif

typedef enum {
    APP_QZB_OFFLINE, APP_QZB_IDLE, APP_QZB_CONFIG,
    APP_QZB_ENABLING, APP_QZB_RUN, APP_QZB_FAULT
} app_motorQZB_phase_t;
typedef enum {
    APP_QZB_FAULT_NONE, APP_QZB_FAULT_OFFLINE, APP_QZB_FAULT_MOTOR,
    APP_QZB_FAULT_BAD_MODE, APP_QZB_FAULT_BAD_COMMAND, APP_QZB_FAULT_PARAM_TIMEOUT,
    APP_QZB_FAULT_MODE_MISMATCH, APP_QZB_FAULT_ENABLE_TIMEOUT,
    APP_QZB_FAULT_DISABLE_TIMEOUT, APP_QZB_FAULT_LOST_ENABLE,
    APP_QZB_FAULT_TX, APP_QZB_FAULT_KILLED
} app_motorQZB_fault_t;
typedef struct {
    uint8_t enable, mode;       /* 输入：mode=1 力矩、2 速度、3 位置。 */
    float tq, vel, pos;         /* 输入：Nm、rad/s、rad；所有浮点输入（含未使用项）必须有限。 */
    float pos_kp, pos_kd;       /* 输入：位置模式 MIT 增益，发送时应用位置软限幅。 */
    float vel_kd;               /* 输入：速度模式 MIT 阻尼增益 Nm/(rad/s)。 */
    uint8_t clear_error;        /* 上电默认挂起一次，遥控复位/失联放开后重连再次置位；失能时清错成功发送后自动清零。 */
    app_motorQZB_phase_t phase;
    app_motorQZB_fault_t fault;
    mwQZB_mode_t active_mode;      /* 输出：已确认的参数 11，0 未知、2 MIT；三种应用模式共用 MIT。 */
    mwQZB_feedback_t fb;
    uint8_t online;
    uint32_t rx_count;
    uint32_t tx_busy;          /* 输出：发送 FIFO 满而推迟重试的累计次数，仅诊断。 */
} app_motorQZB_t;

extern volatile app_motorQZB_t g_motorQZB[APP_MOTORQZB_COUNT];
extern volatile uint8_t g_motorQZB_kill;
/* 启动期调用，返回 1；内部直接接入 g_motorSHS_kill，不修改共享急停值。 */
int app_motorQZB_init(void);
void app_motorQZB_tick(void);
/* 临界区内请求四轮清错，复用 clear_error 一次性机制：仅 enable=0 且反馈失能
 * 时发送 0xFB，成功发出后自动清零；离线时保持挂起，待上线后发送。 */
void app_motorQZB_request_clear(void);
/* 仅置位/释放本模块急停；kill 后立即 release 的事件仍由下一拍处理。
 * 共享 g_motorSHS_kill 或直接写 g_motorQZB_kill 须保持至少一拍；
 * 解除后仍须观察 enable=0，再接受 enable=1。 */
void app_motorQZB_kill(void);
void app_motorQZB_kill_release(void);
#endif
