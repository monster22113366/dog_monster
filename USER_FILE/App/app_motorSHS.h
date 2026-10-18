/*
 * 本文件属于 App 应用层，调用 mw_canSHS 协议层发送命令并读取一致反馈快照。
 * 对外提供电机控制状态、运行阶段、故障定义及初始化和周期调度接口。
 * 系统初始化时由上层调用 app_motorSHS_init，随后由 app_ctrl 周期调用 app_motorSHS_tick。
 * 上层通过 g_motorSHS[i] 写入使能、模式和目标，并读取反馈、阶段及故障状态；g_motorSHS[i].mode 取 1~3。
 */
#ifndef APP_MOTORSHS_H
#define APP_MOTORSHS_H

#include "app_log.h"
#include "mw_canSHS.h"
#include <stdint.h>
#ifndef APP_MOTORSHS_PERIOD_MS
/* 电机状态机调度周期，单位毫秒；由 app_ctrl 按此周期调用 app_motorSHS_tick。 */
#define APP_MOTORSHS_PERIOD_MS 2u
#endif
#ifndef APP_MOTORSHS_NUM
/* 受控电机数量，必须与 app_motorSHS.c 中 motors[] 表长度一致。 */
#define APP_MOTORSHS_NUM 8u
#endif
/* 数值就是 0x00B 的 B0 控制模式；0（电压）不支持，因为协议里没有电压设定指令，传 0 会按非法模式报 BAD_MODE。
 * 需要按电流给定时，由上层按电机 KT 换算成力矩，再用 TORQUE 模式。
 */
typedef enum {
    APP_SHS_MODE_TORQUE = MWSHS_CTRL_TORQUE,   /* 1，target单位：Nm */
    APP_SHS_MODE_VEL = MWSHS_CTRL_VEL,         /* 2，target单位：turns/s（转子侧） */
    APP_SHS_MODE_POS = MWSHS_CTRL_POS          /* 3，target单位：turns（转子侧） */
} app_motorSHS_mode_t;
typedef enum {
    APP_SHS_PH_OFFLINE, /* 离线：尚无近期有效帧；收到新帧后进入空闲。 */
    APP_SHS_PH_IDLE,    /* 空闲：电调保持待机；使能已重新置位时开始配置。 */
    APP_SHS_PH_CONFIG,  /* 配置：发送模式和限值设置；同一个 tick 内就转到 WAIT_CL，调试器里通常看不到。 */
    APP_SHS_PH_WAIT_CL, /* 等待闭环：等待反馈状态变为闭环，超时则故障。 */
    APP_SHS_PH_RUN,     /* 运行：周期发送目标；失能、通信异常或模式改变时离开。 */
    APP_SHS_PH_SWITCH,  /* 切换：已请求电调进入空闲，等待心跳确认 state==IDLE 后再按新模式配置；超时报 SWITCH_TIMEOUT。 */
    APP_SHS_PH_FAULT    /* 故障：请求电调空闲；使能回到零后允许重新使能。 */
} app_motorSHS_phase_t;
typedef enum {
    APP_SHS_FAULT_NONE,           /* 无故障：初始化时或成功进入运行后。 */
    APP_SHS_FAULT_CL_TIMEOUT,     /* 闭环确认超时：配置后未及时反馈闭环。 */
    APP_SHS_FAULT_LOST_CL,        /* 闭环丢失：运行中反馈状态不再是闭环。 */
    APP_SHS_FAULT_AXIS_ERROR,     /* 电机轴故障：反馈中的轴错误码非零。 */
    APP_SHS_FAULT_SWITCH_TIMEOUT, /* 模式切换超时：电调未及时确认进入空闲。 */
    APP_SHS_FAULT_BAD_MODE,       /* 模式无效：收到范围之外的模式值。 */
    APP_SHS_FAULT_KILLED          /* 一键失能：g_motorSHS_kill 置位，全部电机急停并锁存。 */
} app_motorSHS_fault_t;
typedef struct {
    volatile uint8_t enable;       /* 输入：0 禁止、1 请求运行；调试器或上层写，状态机读。 */
    volatile app_motorSHS_mode_t mode;      /* 输入：请求控制模式；调试器或上层写，状态机读。 */
    volatile float target;         /* 输入：目标值，单位由 mode 决定；调试器或上层写，状态机读。 */
    volatile app_motorSHS_phase_t phase;    /* 输出：当前应用阶段；状态机写，调试器或上层读。 */
    volatile app_motorSHS_mode_t active_mode; /* 输出：电调当前已配置模式；状态机写，调试器或上层读。 */
    volatile mwSHS_motor_t fb;        /* 输出：协议层整理的最新反馈；状态机写，调试器或上层读。 */
    volatile app_motorSHS_fault_t fault;    /* 输出：最近故障原因；状态机写，调试器或上层读。 */
} app_motorSHS_state_t;
extern volatile app_motorSHS_state_t g_motorSHS[APP_MOTORSHS_NUM];
extern const sim_vars_t g_motorSHS_vars;
/* 输入：1 = 一键失能（调试器 Live Watch 可直接写），0 = 解除；解除后需 enable 先回 0 再置 1 才会重新使能。 */
extern volatile uint8_t g_motorSHS_kill;
/* app_motorSHS_init：初始化电机应用状态和状态机
 * 说明：系统启动时由上层调用一次。
 */
void app_motorSHS_init(void);
/* app_motorSHS_kill：置位 g_motorSHS_kill，一键失能
 * 说明：只写标志不发帧，可在任意上下文（含按键中断）调用；下一拍 app_motorSHS_tick 向全部电机发 0x002 急停。
 */
void app_motorSHS_kill(void);
/* app_motorSHS_kill_release：清除 g_motorSHS_kill
 * 说明：只写标志；app_rc 在 TIM6 中先于 app_motorSHS_tick 调用，主循环也可调用。
 */
void app_motorSHS_kill_release(void);
/* app_motorSHS_tick：采样反馈并推进全部电机控制状态机
 * 参数：无。
 * 说明：由调度器按 APP_MOTORSHS_PERIOD_MS 周期调用，内部通过 HAL_GetTick 获取当前时间；每次调用依次推进全部电机的状态机。失能时先群发急停。
 */
void app_motorSHS_tick(void);
/* g_motor_all_running：返回全部电机是否均处于 APP_SHS_PH_RUN，供记录帧 flags 使用。 */
uint8_t g_motor_all_running(void);
void motor_control_test(void);
#endif
