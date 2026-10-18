/*
 * gait 意为步态
 * 本文件属于 Componnet 算法组件层，实现摆动段平滑、支撑段匀速的小跑足端轨迹。
 * +z 向下，因此抬腿减小 z；周期边界及摆动/支撑切换处位置连续。
 * 仅依赖 gait.h 和 <math.h>；无硬件依赖，参数与输出均由调用者提供。
 */
#include "gait.h"
#include <math.h>

/* gait_s5：执行限幅后的五次平滑插值
 * 参数：s 为归一化进度
 * 返回：[0,1] 内的平滑进度
 * 说明：输入低于 0 或高于 1 时分别返回端点值。
 */
float gait_s5(float s) {
    s = s < 0.0f ? 0.0f : (s > 1.0f ? 1.0f : s);
    return s * s * s * (s * (6.0f * s - 15.0f) + 10.0f);
}

/* gait_trot_foot：按摆动和支撑分段计算足端位置
 * 参数：g 为步态参数；phase 为任意有限周期相位；p 接收 {x,z}
 * 返回：无
 * 说明：摆动占前半周期，支撑占后半周期；apex 限幅防止插值分母为零。
 */
void gait_trot_foot(const gait_param_t *g, float phase, float p[2]) {
    if (g == 0 || p == 0)
        return;

    phase -= floorf(phase);
    const float apex = g->apex < 0.05f ? 0.05f : (g->apex > 0.95f ? 0.95f : g->apex);
    if (phase < 0.5f) {
        const float tau = phase / 0.5f;
        const float lift = tau <= apex ? gait_s5(tau / apex)
                                      : 1.0f - gait_s5((tau - apex) / (1.0f - apex));
        p[0] = -g->stride / 2.0f + g->stride * gait_s5(tau);
        p[1] = g->z0 - g->height * lift;
    } else {
        const float tau = (phase - 0.5f) / 0.5f;
        p[0] = g->stride / 2.0f - g->stride * tau;
        p[1] = g->z0;
    }
}

/* gait_slew：限制目标范围和单拍变化量，不越过目标。 */
float gait_slew(float cur, float target, float step, float lo, float hi) {
    if (!isfinite(cur) || !isfinite(target) || !isfinite(step) ||
        !isfinite(lo) || !isfinite(hi) || step <= 0.0f || lo > hi)
        return cur;
    target = fmaxf(lo, fminf(hi, target));
    return cur + fmaxf(-step, fminf(step, target - cur));
}
