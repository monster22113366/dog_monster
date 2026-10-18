/*
 * 本文件属于 App 应用层，组织 TIM6 驱动的 500 Hz 控制链。
 * 依赖 Bsp 定时器接口及 IMU、遥控、整机、电机、通用记录器任务，不直接访问硬件。
 * 各任务只在 TIM6 中断中执行；初始化中的硬件延时在启动定时器前完成。
 */
#include "app_ctrl.h"
#include "app_imu.h"
#include "app_rc.h"
#include "app_legs.h"
#include "app_chassis.h"
#include "app_motorSHS.h"
#include "bsp_tim.h"
#include <string.h>

typedef struct {
    void (*task)(void);
    uint8_t div;
} ctrl_task_t;

static const ctrl_task_t tasks[] = {
    {app_imu_tick, 1u},
    {app_rc_tick, 5u},
    {app_legs_tick, 1u},
    {app_chassis_tick, 1u},
    {app_motorSHS_tick, 1u},
    {app_log_tick, 1u},
};
_Static_assert(sizeof(tasks) / sizeof(tasks[0]) == APP_CTRL_TASK_NUM,
               "tasks must match APP_CTRL_TASK_NUM");
static uint8_t s_div_count[sizeof(tasks) / sizeof(tasks[0])];
static volatile uint32_t app_ctrl_ms_cnt;
volatile ctrl_debug_t g_ctrl;

/* 记录器只访问非 volatile 影子量；本拍末尾更新，帧内对应上一拍统计。 */
static uint16_t s_ctrl_exec_u16;    //本拍CNT差的快照地址
static uint16_t s_ctrl_overrun_u16; //CNT差超过周期的次数的快照地址
static uint16_t s_ctrl_tick_u16;
static uint16_t s_ctrl_task_us[APP_CTRL_TASK_NUM];
static const sim_var_desc_t s_ctrl_vars_a[] = {
    {"exec_us", SIM_VAR_U16, 0u, &s_ctrl_exec_u16},
    {"overrun_count", SIM_VAR_U16, 0u, &s_ctrl_overrun_u16},
    {"tick_index", SIM_VAR_U16, 0u, &s_ctrl_tick_u16},
    {"task_us", SIM_VAR_U16, APP_CTRL_TASK_NUM, s_ctrl_task_us},
};
const sim_vars_t g_ctrl_vars = {"ctrl", s_ctrl_vars_a, sizeof(s_ctrl_vars_a) / sizeof(s_ctrl_vars_a[0])};

/* elapsed_us：读取当前计数，差为负时加回计数器周期；不能检出完整周期的超时。 */
static inline uint32_t elapsed_us(uint32_t t0) {
    const int32_t raw = (int32_t)(bsp_ctrl_timer_us() - t0);
    return (uint32_t)(raw < 0 ? raw + (int32_t)bsp_ctrl_timer_range_us() : raw);
}

/* app_ctrl_init：先复位时间，再保留各模块原有初始化语义，最后初始化记录器。 */
void app_ctrl_init(void) {
    app_ctrl_ms_cnt = 0u;
    memset((void *)&g_ctrl, 0, sizeof(g_ctrl));
    memset(s_div_count, 0, sizeof(s_div_count));
    s_ctrl_exec_u16 = 0u;
    s_ctrl_overrun_u16 = 0u;
    s_ctrl_tick_u16 = 0u;
    memset(s_ctrl_task_us, 0, sizeof(s_ctrl_task_us));
    app_imu_init();
    app_legs_init();
    app_chassis_init();
    /* 腿/轮电机已在 app_can_attach() 中于 CAN 启动前初始化，这里不得重复。 */
    app_rc_init();
    app_log_init();
}

/* app_ctrl_start：登记控制入口，仅由初始化流程调用一次。 */
void app_ctrl_start(void) {
    g_ctrl.start_err = bsp_ctrl_timer_start(app_ctrl_tick) != 0;
}

/* app_ctrl_tick：依次执行到期任务，拍末发布统计和下一拍的统一时间。
 * 耗时按 1 MHz 自由计数器读两次求差；调用方在差为负时加回计数器周期，跨回绕同样正确。
 */
void app_ctrl_tick(void) {
    const uint32_t t0 = bsp_ctrl_timer_us();
    for (uint32_t i = 0u; i < sizeof(tasks) / sizeof(tasks[0]); i++) {
        g_ctrl.task_us[i] = 0u;
        s_div_count[i]++;
        if (s_div_count[i] >= tasks[i].div) {
            s_div_count[i] = 0u;
            const uint32_t task_t0 = bsp_ctrl_timer_us();
            tasks[i].task();
            const uint16_t task_elapsed = (uint16_t)elapsed_us(task_t0);
            g_ctrl.task_us[i] = task_elapsed;
            if (task_elapsed > g_ctrl.task_max_us[i])
                g_ctrl.task_max_us[i] = task_elapsed;
        }
    }
    const uint32_t elapsed = elapsed_us(t0);
    g_ctrl.exec_us = elapsed;
    if (elapsed > g_ctrl.exec_max_us)
        g_ctrl.exec_max_us = elapsed;
    if (elapsed > APP_CTRL_PERIOD_MS * 1000u)
        g_ctrl.overrun_count++;
    g_ctrl.tick_count++;
    app_ctrl_ms_cnt += APP_CTRL_PERIOD_MS;
    g_ctrl.ms = app_ctrl_ms_cnt;
    s_ctrl_exec_u16 = (uint16_t)g_ctrl.exec_us;
    s_ctrl_overrun_u16 = (uint16_t)g_ctrl.overrun_count;
    s_ctrl_tick_u16 = (uint16_t)g_ctrl.tick_count;
    for (uint32_t i = 0u; i < APP_CTRL_TASK_NUM; i++)
        s_ctrl_task_us[i] = g_ctrl.task_us[i];
}

/* app_ctrl_ms：对齐的 uint32_t 仅作一次 volatile 读取，不屏蔽中断。 */
uint32_t app_ctrl_ms(void) {
    return app_ctrl_ms_cnt;
}
