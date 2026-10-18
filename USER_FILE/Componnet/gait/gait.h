/*
 * 本文件属于 Componnet 算法组件层，定义单足的对角小跑周期轨迹。
 * 相位前半段摆动、后半段支撑；各腿相位偏移由上层分配。
 * 腿坐标系 +x 向前、+z 向下，抬腿时从 z0 减去高度；长度单位 m。
 * x 在相位 0 和 0.5 处连续；支撑段 x 匀速回摆，不保证切换点速度连续。
 * 头文件无外部依赖；实现仅依赖 <math.h>，无 HAL 依赖及可变全局状态。
 */
#ifndef COMPONENT_GAIT_H
#define COMPONENT_GAIT_H

typedef struct {
    float stride, height, z0; /* 步长、抬腿高度及支撑高度，单位 m */
    float apex;              /* 抬腿最高点在摆动段中的位置比例，例如 0.32 */
} gait_param_t;

/* gait_s5：计算五次平滑插值
 * 参数：s 为归一化进度
 * 返回：s^3*(6*s^2-15*s+10)
 * 说明：先将 s 限制到 [0,1]，两端的一阶和二阶导数均为零。
 */
float gait_s5(float s);

/* gait_trot_foot：生成给定相位的单足目标位置
 * 参数：g 为步态参数；phase 为周期相位；p 接收 {x,z}，单位 m
 * 返回：无
 * 说明：相位用 floorf 回绕到 [0,1)，apex 限幅至 [0.05,0.95]；空指针不操作。
 */
void gait_trot_foot(const gait_param_t *g, float phase, float p[2]);

/* 目标先限幅，再按非负步长靠拢；无效参数或非有限目标保持当前值。 */
float gait_slew(float cur, float target, float step, float lo, float hi);

#endif
