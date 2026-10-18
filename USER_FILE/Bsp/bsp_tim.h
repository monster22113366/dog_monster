/*
 * 本文件属于 Bsp 硬件层，声明控制定时器接口，依赖标准整数类型。
 * 本层只做定时器与回调登记，不含业务；回调在中断上下文执行，必须极短。
 */
#ifndef BSP_TIM_H
#define BSP_TIM_H

#include <stdint.h>

/* bsp_ctrl_timer_start：登记回调并启动 TIM6 更新中断，成功返回 0，失败返回 -1。 */
int bsp_ctrl_timer_start(void (*cb)(void));

/* bsp_ctrl_timer_us：返回本拍开始后的微秒计数，范围 0~1999，每 2 ms 回绕。 */
uint32_t bsp_ctrl_timer_us(void);

/* bsp_ctrl_timer_range_us：返回计数器满量程微秒数（ARR + 1），用于处理 CNT 回绕
 * 参数：无
 * 返回：满量程 us；未初始化时返回 0
 */
uint32_t bsp_ctrl_timer_range_us(void);

#endif
