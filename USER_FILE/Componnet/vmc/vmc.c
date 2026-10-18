/*
 * vmc是virtual_mass_control的缩写，是一种控制算法，意为二维足端虚拟弹簧阻尼
 * 本文件属于 Componnet 算法组件层，实现二维足端虚拟弹簧阻尼及力矩映射。
 * 足端力按对地施力定义；通过 tau=J^T*f 与 v=J*qd 保持虚功一致。
 * 仅依赖 vmc.h 和 <math.h>；雅可比由调用者提供，无腿模型耦合或全局状态。
 */
#include "vmc.h"
#include <math.h>

/* vmc_force：计算并限幅两轴足端对地力
 * 参数：k 为增益、前馈及限幅；p_ref、v_ref 为目标；p、v 为反馈；f 为输出
 * 返回：无
 * 说明：NaN 输入引起的力或 NaN 限幅均将该轴置零；任一空指针不操作。
 */
void vmc_force(const vmc_gain_t *k, const float p_ref[2], const float v_ref[2],
               const float p[2], const float v[2], float f[2]) {
    if (k == 0 || p_ref == 0 || v_ref == 0 || p == 0 || v == 0 || f == 0)
        return;

    for (int i = 0; i < 2; ++i) {
        float force = k->kp[i] * (p_ref[i] - p[i])
                    + k->kd[i] * (v_ref[i] - v[i]) + k->ff[i];
        const float limit = k->f_max[i];
        if (isnan(force) || isnan(limit))
            force = 0.0f;
        else
            force = force < -limit ? -limit : (force > limit ? limit : force);
        f[i] = force;
    }
}

/* vmc_torque：按虚功关系将对地力换算为电机力矩
 * 参数：J 为行 {x,z}、列 {qh,qk} 的雅可比；f 为足端力；tau 为力矩输出
 * 返回：无
 * 说明：先计算两个分量再写出，支持输入输出数组重叠；空指针不操作。
 */
void vmc_torque(const float J[2][2], const float f[2], float tau[2]) {
    if (J == 0 || f == 0 || tau == 0)
        return;
    const float tau_h = J[0][0] * f[0] + J[1][0] * f[1];
    const float tau_k = J[0][1] * f[0] + J[1][1] * f[1];
    tau[0] = tau_h;
    tau[1] = tau_k;
}

/* vmc_foot_vel：计算雅可比与电机角速度的乘积
 * 参数：J 为位置雅可比；qd 为电机角速度；v 为足端速度输出
 * 返回：无
 * 说明：先计算两个分量再写出，支持输入输出数组重叠；空指针不操作。
 */
void vmc_foot_vel(const float J[2][2], const float qd[2], float v[2]) {
    if (J == 0 || qd == 0 || v == 0)
        return;
    const float vx = J[0][0] * qd[0] + J[0][1] * qd[1];
    const float vz = J[1][0] * qd[0] + J[1][1] * qd[1];
    v[0] = vx;
    v[1] = vz;
}
