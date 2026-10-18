/*
 * 本文件属于 Bsp 硬件层，依赖 tim.h 的 HAL 接口及 CubeMX 生成的 htim6。
 * 本层只做定时器与回调登记，不含业务；回调在中断上下文执行，必须极短。
 * TIM6 时钟 100 MHz，PSC=99、ARR=1999，计数单位 1 us，更新周期 2 ms。
 */
#include "bsp_tim.h"
#include "tim.h"

static void (*s_ctrl_cb)(void);

/* bsp_ctrl_timer_start：登记回调，清除初始化产生的更新标志后启动周期中断。 */
int bsp_ctrl_timer_start(void (*cb)(void)) {
    s_ctrl_cb = cb;
    __HAL_TIM_SET_COUNTER(&htim6, 0u);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
        return -1;
    return 0;
}

/* bsp_ctrl_timer_us：读取 TIM6 当前拍内计数，不扩展硬件回绕。 */
uint32_t bsp_ctrl_timer_us(void) {
    /* TIM6 的 PSC=99，计数频率 1 MHz，CNT 每 2000 跳过回绕；调用方按 cnt_range_us() 处理回绕。 */
    return (uint32_t)TIM6->CNT;
}

/* bsp_ctrl_timer_range_us：计数器满量程对应的微秒数，等于 ARR + 1。 */
uint32_t bsp_ctrl_timer_range_us(void) {
    return (uint32_t)(htim6.Init.Period + 1u);
}

/* HAL_TIM_PeriodElapsedCallback：仅转发 TIM6 更新事件，忽略其他定时器。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6 && s_ctrl_cb != 0)
        s_ctrl_cb();
}
