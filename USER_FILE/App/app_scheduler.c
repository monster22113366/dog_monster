/*
 * 本文件属于 App 应用层，依赖 app_scheduler.h 的 HAL 时基与 app_ctrl 控制入口。
 * 主循环仅调度慢任务；500 Hz 控制链见 app_ctrl，由 TIM6 中断执行。
 */
#include "app_scheduler.h"
#include "app_ctrl.h"
#include "app_motorQZB.h"
#include "app_motorSHS.h"



/********** 全局变量 **********/
static uint8_t task_num;
/* 轮电机基础控制在主循环每 2 ms 执行，默认失能。 */
static task_t tasks[] = {{app_motorQZB_tick, 2u, 0}};
/********** 全局变量 **********/


/********** 1-外设数量初始化函数 **********/
/* task_init：初始化周期任务
 * 参数：无
 * 返回：无
 * 说明：GPIO、SPI2、CAN 和 TIM6 就绪后初始化控制链，再启动定时器。
 */
/* 仅启动期、CAN 硬件启动前调用一次。重复调用会清空电机状态及接收缓存，
 * 并重新挂起轮电机清错；运行期间不得重新调用。 */
void app_can_attach(void)
{
    app_motorSHS_init();
    app_motorQZB_init();
}

void task_init(void)
{
    task_num = (uint8_t)(sizeof(tasks) / sizeof(tasks[0]));
    app_ctrl_init();
    app_ctrl_start();
}
/********** 1-外设数量初始化函数 **********/


/********** 3-外设调用函数 **********/
/* task_run：主循环按 HAL 毫秒时基执行到期慢任务，控制链由 TIM6 独立驱动。 */
void task_run(void)
{
    for(uint8_t i = 0; i < task_num; i++)
    {
        uint32_t now_time = HAL_GetTick();
        if(now_time - tasks[i].task_last_runtime >= tasks[i].task_period)
        {
            tasks[i].task_last_runtime = now_time;
            tasks[i].task();
        }
    }
}
/********** 3-外设调用函数 **********/
