/**
 * @file app_scheduler.h
 * @brief 应用层主头文件 - 任务调度系统
 *
 * 提供简易的协作式任务调度器，支持周期性任务执行。
 *
 * @example 添加新任务
 * // 1. 在 tasks[] 数组中添加任务条目
 * task_t tasks[] = {
 *     {my_task, 100, 0},  // 每100ms执行一次
 * };
 *
 * // 2. 在 task_init() 中调用初始化函数
 * void task_init() {
 *     my_module_init();
 * }
 */

#ifndef APP_SCHEDULER_H
#define APP_SCHEDULER_H
#include "stm32h7xx_hal.h"

/**
 * @brief 任务结构体
 */
typedef struct
{
  void (*task)(void);            ///< 任务函数指针
  uint16_t task_period;       ///< 任务执行周期(ms)
  uint32_t task_last_runtime; ///< 上次执行时间戳
}task_t;

#ifdef __cplusplus
extern "C" {
#endif

/* CAN 启动前登记全部协议适配；仅调用一次，重复调用会清状态。 */
void app_can_attach(void);
void task_init(void);
void task_run(void);

#ifdef __cplusplus
}
#endif

#endif //APP_SCHEDULER_H