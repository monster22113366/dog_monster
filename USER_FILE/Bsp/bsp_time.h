/* 通用平台毫秒时基，沿用 HAL tick，允许 uint32_t 回绕。 */
#ifndef BSP_TIME_H
#define BSP_TIME_H
#include "stm32h7xx_hal.h"
static inline uint32_t bsp_millis(void) { return HAL_GetTick(); }
#endif
