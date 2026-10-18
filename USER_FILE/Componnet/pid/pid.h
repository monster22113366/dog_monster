/*
 * 本文件属于 Componnet 算法组件层，定义带积分限幅和测量微分的 PID 接口。
 * 对外提供初始化、状态复位和单步计算；由上层控制循环传入独立实例。
 * 仅依赖 <stdint.h>；无 HAL 依赖及可变全局状态，时间单位为秒。
 */
#ifndef COMPONENT_PID_H
#define COMPONENT_PID_H

#include <stdint.h>

typedef struct {
    float kp, ki, kd;   /* 增益 */
    float i_max;        /* 积分项绝对值上限，输出单位 */
    float out_max;      /* 输出绝对值上限 */
    float i_out;        /* 积分项累计值，输出单位 */
    float last_fb;      /* 上一次反馈，用于测量微分 */
    uint8_t first;      /* 1 表示尚未计算过，首拍微分记 0 */
} pid_ctrl_t;

/* pid_init：设置增益及限幅并复位状态
 * 参数：p 为实例；kp、ki、kd 为增益；i_max、out_max 为非负绝对值上限
 * 返回：无
 * 说明：参数赋值后调用 pid_reset；p 为空时不操作。
 */
void pid_init(pid_ctrl_t *p, float kp, float ki, float kd,
              float i_max, float out_max);

/* pid_reset：清除积分和反馈历史
 * 参数：p 为实例
 * 返回：无
 * 说明：保留增益及限幅；i_out、last_fb 置零，first 置 1；空指针不操作。
 */
void pid_reset(pid_ctrl_t *p);

/* pid_calc：计算一拍 PID 输出
 * 参数：p 为实例；ref 为目标；fb 为反馈；dt 为采样间隔，单位秒
 * 返回：限幅后的输出；p 为空时返回 0
 * 说明：微分作用于测量值；dt<=0 时仅计算比例加原积分，不更新任何状态。
 */
float pid_calc(pid_ctrl_t *p, float ref, float fb, float dt);

#endif
