/*
 * 本文件属于 App 应用层，声明 TIM6 控制链入口与统一控制时基。
 * 依赖 app_log.h 的描述符类型；初始化在主循环启动前完成，tick 在中断中执行。
 */
#ifndef APP_CTRL_H
#define APP_CTRL_H

#include "app_log.h"

#ifndef APP_CTRL_PERIOD_MS
#define APP_CTRL_PERIOD_MS 2u
#endif

/* 任务数由 app_ctrl.c 按任务表 sizeof 静态校验。 */
#define APP_CTRL_TASK_NUM 6u

typedef struct {
    uint8_t start_err;          /* 输出：1 = 定时器启动失败。 */
    uint32_t exec_us;           /* 输出：本拍 CNT 差，跨硬件回绕时不可靠。 */
    uint32_t exec_max_us;       /* 输出：历史最大 CNT 差。 */
    volatile uint16_t task_us[APP_CTRL_TASK_NUM];     /* 输出：各任务本拍耗时 us，未执行为 0。 */
    volatile uint16_t task_max_us[APP_CTRL_TASK_NUM]; /* 输出：各任务历史最大耗时 us。 */
    uint32_t overrun_count;     /* 输出：CNT 差超过周期的次数，不能保证检出跨拍超时。 */
    uint32_t tick_count;        /* 输出：已完成控制拍数。 */
    uint32_t ms;                /* 输出：控制节拍 ms，与 app_ctrl_ms 同值。 */
} ctrl_debug_t;

extern volatile ctrl_debug_t g_ctrl;
extern const sim_vars_t g_ctrl_vars;

/* app_ctrl_init：定时器启动前复位控制状态，依次初始化 IMU、整机、轮底盘、电机、遥控与记录器。 */
void app_ctrl_init(void);

/* app_ctrl_start：启动 TIM6 控制链，结果写入 g_ctrl.start_err。 */
void app_ctrl_start(void);

/* app_ctrl_tick：在 TIM6 中断中按任务表分频执行，并更新控制时基及统计。 */
void app_ctrl_tick(void);

/* app_ctrl_ms：无锁读取 32 位控制节拍 ms；被中断打断时允许返回最多晚 2 ms 的值。 */
uint32_t app_ctrl_ms(void);

#endif
