/*
 * pid是一种控制策略，基于增益、积分和微分，用于控制系统的输出。
 * 本文件属于 Componnet 算法组件层，实现与硬件无关的单精度 PID。
 * 积分项以输出单位累计；测量微分避免目标阶跃引起微分冲击。
 * 仅依赖 pid.h；所有历史状态保存在调用者提供的 pid_ctrl_t 中。
 */
#include "pid.h"

/* pid_clamp：对数值作对称限幅
 * 参数：value 为待限幅值；limit 为非负绝对值上限
 * 返回：截取到 [-limit, limit] 的值
 * 说明：内部纯函数，不保存状态。
 */
static float pid_clamp(float value, float limit) {
    return value < -limit ? -limit : (value > limit ? limit : value);
}

/* pid_init：设置控制参数并初始化动态状态
 * 参数：p 为实例；kp、ki、kd 为增益；i_max、out_max 为非负限幅值
 * 返回：无
 * 说明：先设置全部参数，再调用 pid_reset；空指针不操作。
 */
void pid_init(pid_ctrl_t *p, float kp, float ki, float kd,
              float i_max, float out_max) {
    if (p == 0)
        return;
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->i_max = i_max;
    p->out_max = out_max;
    pid_reset(p);
}

/* pid_reset：复位控制器的积分和反馈历史
 * 参数：p 为实例
 * 返回：无
 * 说明：不改变增益或限幅；复位后的首拍微分为零。
 */
void pid_reset(pid_ctrl_t *p) {
    if (p == 0)
        return;
    p->i_out = 0.0f;
    p->last_fb = 0.0f;
    p->first = 1u;
}

/* pid_calc：执行比例、积分、测量微分及输出限幅
 * 参数：p 为实例；ref 为目标；fb 为反馈；dt 为采样间隔，单位秒
 * 返回：限幅后的控制输出；空指针返回 0
 * 说明：非正 dt 不更新状态；有效采样先积分限幅，再计算测量微分。
 */
float pid_calc(pid_ctrl_t *p, float ref, float fb, float dt) {
    if (p == 0)
        return 0.0f;

    const float err = ref - fb;
    if (dt <= 0.0f)
        return pid_clamp(p->kp * err + p->i_out, p->out_max);

    p->i_out = pid_clamp(p->i_out + p->ki * err * dt, p->i_max);
    const float d = p->first ? 0.0f : -p->kd * (fb - p->last_fb) / dt;
    const float out = pid_clamp(p->kp * err + p->i_out + d, p->out_max);
    p->last_fb = fb;
    p->first = 0u;
    return out;
}
