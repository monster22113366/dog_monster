/* critical 意为裸机短临界区：保存并恢复进入前的中断屏蔽状态。 */
#ifndef BSP_CRITICAL_H
#define BSP_CRITICAL_H
#include "stm32h7xx.h"

static inline uint32_t bsp_critical_enter(void) {
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    return key;
}

static inline void bsp_critical_leave(uint32_t key) {
    if (key == 0u) __enable_irq();
}
#endif
