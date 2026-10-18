/*
 * 本文件属于 Componnet 算法组件层，定义差速轮速混控、斜坡及步态相位缩放。
 * 无硬件依赖；实现仅依赖 <math.h>，轮序为 LF、RF、LB、RB。
 */
#ifndef WHEEL_MIX_H
#define WHEEL_MIX_H

typedef struct { float fwd; float yaw; float vmax; } wheel_cmd_t;

/* wheel_mix：归一化前进/转向输入，输出车体前进为正的输出侧轮速 rad/s。 */
void wheel_mix(const wheel_cmd_t *c, float out[4]);
/* wheel_ramp：按 acc/dec（rad/s²）和 dt（s）限速；反向先减至零。 */
float wheel_ramp(float cur, float target, float acc, float dec, float dt);
/* wheel_phase_scale：前半周期摆动、后半周期支撑，边界以宽度 blend 对称过渡。 */
float wheel_phase_scale(float phase, float blend);

#endif
