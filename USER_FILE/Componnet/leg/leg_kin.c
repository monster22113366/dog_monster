/*
 * leg_kin是leg_kinematics的缩写，意为腿部运动学
 * 本文件属于 Componnet 算法组件层，实现同轴双髋腿的正解、逆解和解析雅可比。
 * 坐标及分支约定见 leg_kin.h；所有运算使用 float，状态由调用者持有。
 * 仅依赖 leg_kin.h 和 <math.h>，不访问硬件或其他控制组件。
 */
#include "leg_kin.h"
#include <math.h>

/* leg_fk：全称为 leg_forward_kinematics，意为足端正解，用于计算给定电机角下的足端坐标
 * 参数：g 为几何参数；qh、qk 为电机角；p 接收 {x, z}
 * 返回：成功为 0；空指针或根号内为负时为 -1
 * 说明：先验证根号定义域，失败时保留调用者原有输出。
 */
int leg_fk(const leg_geom_t *g, float qh, float qk, float p[2]) {
    if (g == 0 || p == 0)
        return -1;

    const float gamma = 0.5f * (qh + qk);
    const float psi = 0.5f * (qk - qh);
    const float sp = sinf(psi);
    const float radicand = g->l2 * g->l2 - g->l1 * g->l1 * sp * sp;
    if (radicand < 0.0f)
        return -1;

    const float l = g->l1 * cosf(psi) + sqrtf(radicand);
    p[0] = l * sinf(gamma);
    p[1] = l * cosf(gamma);
    return 0;
}

/* leg_ik：全称为 leg_inverse_kinematics，意为足端逆解，用于计算工作范围内足端位置的电机角
 * 参数：g 为几何参数；p 为 {x, z}；qh、qk 为角度输出指针
 * 返回：成功为 0；参数无效或腿长越界时为 -1
 * 说明：使用 atan2f(x, z) 保持从 +z 起算的角度约定，失败不改写输出。
 */
int leg_ik(const leg_geom_t *g, const float p[2], float *qh, float *qk) {
    if (g == 0 || p == 0 || qh == 0 || qk == 0)
        return -1;

    const float l = sqrtf(p[0] * p[0] + p[1] * p[1]);
    if (l < g->l_min || l > g->l_max || l <= 0.0f || g->l1 <= 0.0f)
        return -1;

    const float gamma = atan2f(p[0], p[1]);
    float c = (l * l + g->l1 * g->l1 - g->l2 * g->l2) / (2.0f * g->l1 * l);
    c = c < -1.0f ? -1.0f : (c > 1.0f ? 1.0f : c);
    const float psi = acosf(c);
    *qh = gamma - psi;
    *qk = gamma + psi;
    return 0;
}

/* leg_jacobian：按链式法则求 dp/dq，用于计算给定电机角下的解析雅可比
 * 参数：g 为几何参数；qh、qk 为电机角；J 接收行 {x,z}、列 {qh,qk}
 * 返回：无
 * 说明：根项非正时以 1e-6 避免除零；奇异附近为数值保护近似值。
 */
void leg_jacobian(const leg_geom_t *g, float qh, float qk, float J[2][2]) {
    if (g == 0 || J == 0)
        return;

    const float gamma = 0.5f * (qh + qk);
    const float psi = 0.5f * (qk - qh);
    const float sp = sinf(psi);
    const float cp = cosf(psi);
    const float sg = sinf(gamma);
    const float cg = cosf(gamma);
    const float radicand = g->l2 * g->l2 - g->l1 * g->l1 * sp * sp;
    const float s = radicand > 0.0f ? sqrtf(radicand) : 1e-6f;
    const float l = g->l1 * cp + s;
    const float dl = -g->l1 * sp * (1.0f + g->l1 * cp / s);
    const float dpg[2] = {l * cg, -l * sg};
    const float dpp[2] = {dl * sg, dl * cg};

    for (int row = 0; row < 2; ++row) {
        J[row][0] = 0.5f * dpg[row] - 0.5f * dpp[row];
        J[row][1] = 0.5f * dpg[row] + 0.5f * dpp[row];
    }
}
