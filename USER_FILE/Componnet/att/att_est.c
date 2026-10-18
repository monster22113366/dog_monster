/*
 * att_est 是 attitude_estimation 的缩写，意为机身姿态估计。
 * 本文件属于 Componnet 算法组件层，实现每轴 {角度, 陀螺零偏} 二状态线性卡尔曼。
 * 坐标 +x 前、+y 左、+z 上，roll 左侧抬起为正、pitch 机头向下为正；使用 ZYX 欧拉率。
 * 仅依赖 att_est.h 和 <math.h>，不访问硬件或其他控制组件；状态由调用者持有。
 */
#include "att_est.h"
#include <math.h>

#ifndef ATT_G
/* 重力加速度，单位 m/s²。 */
#define ATT_G 9.81f
#endif

/* acc_tilt：由比力求倾角
 * 参数：a 为机体系比力；roll、pitch 接收倾角 rad
 * 返回：无
 * 说明：roll=atan2(ay,az)，pitch=atan2(-ax,sqrt(ay²+az²))。
 */
static void acc_tilt(const float a[3], float *roll, float *pitch) {
    *roll = atan2f(a[1], a[2]);
    *pitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));
}

/* acc_ok：检查加速度修正条件
 * 参数：c 为配置；a 为机体系比力
 * 返回：1 可信，0 仅预测
 * 说明：保留蓝本的近零比力保护；acc_gate<=0 时关闭相对 g 的偏差门限。
 */
static int acc_ok(const att_cfg_t *c, const float a[3]) {
    const float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (n < 0.1f * ATT_G)
        return 0;
    return c->acc_gate <= 0.0f || fabsf(n / ATT_G - 1.0f) <= c->acc_gate;
}

/* euler_rate：机体角速度换算为 ZYX 欧拉角速率
 * 参数：roll、pitch 为当前姿态；w 为机体角速度；droll、dpitch 接收角速率 rad/s
 * 返回：无
 * 说明：计算横滚和俯仰分量，不估计航向。
 */
static void euler_rate(float roll, float pitch, const float w[3], float *droll, float *dpitch) {
    const float sr = sinf(roll), cr = cosf(roll), tp = tanf(pitch);
    *droll = w[0] + (sr * w[1] + cr * w[2]) * tp;
    *dpitch = cr * w[1] - sr * w[2];
}

/* kf_axis：单轴 {角度, 零偏} 线性卡尔曼
 * 参数：x、P 为状态及协方差；c 为配置；rate 为未去零偏的欧拉角速率；meas 为倾角；
 *       meas_ok 为加速度可信标志；dt 为间隔 s
 * 返回：更新后的角度 rad
 * 说明：meas_ok 为 0 时只预测，计算顺序与 SIL 蓝本一致。
 */
static float kf_axis(float x[2], float P[2][2], const att_cfg_t *c, float rate, float meas, int meas_ok, float dt) {
    x[0] += (rate - x[1]) * dt;
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + c->q_ang);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += c->q_bias * dt;
    if (meas_ok) {
        const float y = meas - x[0];
        const float S = P[0][0] + c->r;
        const float k0 = P[0][0] / S, k1 = P[1][0] / S;
        x[0] += k0 * y;
        x[1] += k1 * y;
        const float p00 = P[0][0], p01 = P[0][1];
        P[0][0] -= k0 * p00; P[0][1] -= k0 * p01;
        P[1][0] -= k1 * p00; P[1][1] -= k1 * p01;
    }
    return x[0];
}

/* att_init：复位卡尔曼状态与协方差
 * 参数：s 为状态；c 为配置
 * 返回：无
 * 说明：空指针不操作；首样本初始化倾角，航向零偏始终保持 0。
 */
void att_init(att_state_t *s, const att_cfg_t *c) {
    if (s == 0 || c == 0)
        return;
    s->roll = s->pitch = 0.0f;
    for (int i = 0; i < 3; i++)
        s->bias[i] = 0.0f;
    for (int k = 0; k < 2; k++) {
        s->P[k][0][0] = 0.01f; s->P[k][0][1] = s->P[k][1][0] = 0.0f; s->P[k][1][1] = 1e-4f;
    }
    s->first = 1u;
}

/* att_update：处理一个 IMU 样本
 * 参数：s 为状态；c 为配置；gyro、acc 为机体系角速度及比力；dt 为间隔 s
 * 返回：无
 * 说明：空指针、非有限样本或 dt、非正 dt 不更新；首样本用加速度倾角初始化。
 */
void att_update(att_state_t *s, const att_cfg_t *c, const float gyro[3], const float acc[3], float dt) {
    if (s == 0 || c == 0 || gyro == 0 || acc == 0 || !isfinite(dt) || !(dt > 0.0f))
        return;
    for (int i = 0; i < 3; i++) {
        if (!isfinite(gyro[i]) || !isfinite(acc[i]))
            return;
    }
    float ra, pa;
    acc_tilt(acc, &ra, &pa);
    if (s->first) {
        s->roll = ra; s->pitch = pa;
        s->first = 0u;
        return;
    }
    const int ok = acc_ok(c, acc);
    float dr, dp;
    euler_rate(s->roll, s->pitch, gyro, &dr, &dp);
    float xr[2] = {s->roll, s->bias[0]}, xp[2] = {s->pitch, s->bias[1]};
    s->roll = kf_axis(xr, s->P[0], c, dr, ra, ok, dt);
    s->pitch = kf_axis(xp, s->P[1], c, dp, pa, ok, dt);
    s->bias[0] = xr[1];
    s->bias[1] = xp[1];
}
