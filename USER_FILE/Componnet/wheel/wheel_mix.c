/*
 * 本文件属于 Componnet 算法组件层，实现差速轮速混控、斜坡及步态相位缩放。
 * 仅依赖 wheel_mix.h 和 <math.h>；电机方向符号由 App 层处理。
 */
#include "wheel_mix.h"
#include <math.h>

/* s5：限幅后的五次平滑插值，与 gait_s5 的曲线一致。 */
static float s5(float s) {
    s = s < 0.0f ? 0.0f : (s > 1.0f ? 1.0f : s);
    return s * s * s * (s * (6.0f * s - 15.0f) + 10.0f);
}

/* wheel_mix：前进和左右转的混合控制
 * 同比例压缩左右轮目标，非有限输入时全部输出零。 */
void wheel_mix(const wheel_cmd_t *c, float out[4]) {
    if (out == 0)
        return;
    for (unsigned i = 0u; i < 4u; i++)
        out[i] = 0.0f;
    if (c == 0 || !isfinite(c->fwd) || !isfinite(c->yaw) ||
        !isfinite(c->vmax) || c->vmax < 0.0f)
        return;
    const float fwd = fmaxf(-1.0f, fminf(1.0f, c->fwd));
    const float yaw = fmaxf(-1.0f, fminf(1.0f, c->yaw));
    const float left = fwd + yaw;
    const float right = fwd - yaw;
    const float scale = fmaxf(1.0f, fmaxf(fabsf(left), fabsf(right)));
    out[0] = out[2] = (left / scale) * c->vmax;
    out[1] = out[3] = (right / scale) * c->vmax;
}

/* wheel_ramp：斜坡加速/减速
 * 反向本拍只制动至零，下一拍再向目标加速。 */
float wheel_ramp(float cur, float target, float acc, float dec, float dt) {
    if (!isfinite(cur) || !isfinite(target) || !isfinite(acc) ||
        !isfinite(dec) || !isfinite(dt))
        return 0.0f;
    if (dt <= 0.0f || acc < 0.0f || dec < 0.0f)
        return cur;
    const int reversing = (cur > 0.0f && target < 0.0f) ||
                          (cur < 0.0f && target > 0.0f);
    if (reversing)
        target = 0.0f;
    const float rate = fabsf(target) > fabsf(cur) ? acc : dec;
    const float step = rate * dt;
    if (target > cur)
        return cur + fminf(target - cur, step);
    return cur - fminf(cur - target, step);
}

/* wheel_phase_scale：周期环绕后，在 0.5 与 1.0 两侧各 blend/2 平滑切换。 */
float wheel_phase_scale(float phase, float blend) {
    if (!isfinite(phase) || !isfinite(blend))
        return 0.0f;
    phase -= floorf(phase);
    if (blend <= 0.0f)
        return phase >= 0.5f ? 1.0f : 0.0f;
    blend = fminf(blend, 0.5f);
    const float half = blend * 0.5f;
    if (phase < 0.5f - half)
        phase += 1.0f;
    return s5((phase - (0.5f - half)) / blend) *
           s5(((1.0f + half) - phase) / blend);
}
