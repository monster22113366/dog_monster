/*
 * att_est 是 attitude_estimation 的缩写，意为机身姿态估计。
 * 本文件属于 Componnet 算法组件层，定义每轴 {角度, 陀螺零偏} 二状态线性卡尔曼。
 * 输入为机体系样本：gyro 角速度 rad/s，acc 比力 m/s²（静止水平时约 {0, 0, +g}）。
 * 坐标 +x 前、+y 左、+z 上；roll 左侧抬起为正、pitch 机头向下为正，ZYX 欧拉角，单位 rad。
 * 不估计航向；头文件无外部依赖，实现仅依赖自身头文件和 <math.h>，状态由调用者持有。
 */
#ifndef COMPONENT_ATT_EST_H
#define COMPONENT_ATT_EST_H

typedef struct {
    float q_ang, q_bias, r; /* 角度过程噪声 rad²/s、零偏过程噪声 rad²/s³、倾角测量噪声 rad² */
    float acc_gate;        /* 比力模长相对 g 的偏离门限；<=0 关闭门限，近零比力仍不用于修正 */
} att_cfg_t;

typedef struct {
    float roll, pitch;    /* 输出姿态 rad */
    float bias[3];        /* 陀螺零偏估计 rad/s，bias[2] 保持 0，仅为接口统一 */
    float P[2][2][2];     /* 协方差，[轴 0 roll / 1 pitch][2][2]，状态 {角度, 零偏} */
    unsigned char first; /* 1 表示尚未处理样本，首样本用加速度倾角初始化 */
} att_state_t;

/* att_init：复位状态
 * 参数：s 为状态；c 为配置
 * 返回：无
 * 说明：首次 att_update 时用加速度倾角初始化角度；空指针不操作。
 */
void att_init(att_state_t *s, const att_cfg_t *c);

/* att_update：处理一个 IMU 样本
 * 参数：s 为状态；c 为配置；gyro 为机体系角速度 rad/s；acc 为机体系比力 m/s²；dt 为样本间隔 s
 * 返回：无
 * 说明：dt<=0 或样本、dt 非有限时不更新；结果写 s->roll、s->pitch。
 */
void att_update(att_state_t *s, const att_cfg_t *c, const float gyro[3], const float acc[3], float dt);

#endif
