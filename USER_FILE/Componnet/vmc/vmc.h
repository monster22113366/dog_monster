/*
 * 本文件属于 Componnet 算法组件层，定义通用二维虚拟模型控制接口。
 * 下标 0=x、1=z；腿坐标系 +x 向前、+z 向下。
 * f 为足端施加给地面的力，+z 方向推地使机体获得向上的反作用力。
 * J=dp/dq，按虚功关系 tau=J^T*f；电机侧正方向转换由调用者负责。
 * 头文件无外部依赖；实现仅依赖 <math.h>，不依赖 leg_kin.h 或 HAL。
 */
#ifndef COMPONENT_VMC_H
#define COMPONENT_VMC_H

typedef struct {
    float kp[2], kd[2], ff[2], f_max[2]; /* N/m、N*s/m、N、N；下标 0=x、1=z */
} vmc_gain_t;

/* vmc_force：由位置和速度误差计算足端对地力
 * 参数：k 为增益及非负力限幅；p_ref、v_ref 为目标；p、v 为反馈；f 为输出
 * 返回：无
 * 说明：各轴力为 kp*位置误差+kd*速度误差+ff；限幅到 ±f_max，NaN 置零。
 */
void vmc_force(const vmc_gain_t *k, const float p_ref[2], const float v_ref[2],
               const float p[2], const float v[2], float f[2]);

/* vmc_torque：将足端对地力映射为电机力矩
 * 参数：J 为位置雅可比；f 为 {Fx, Fz}，单位 N；tau 接收两个电机力矩，单位 N*m
 * 返回：无
 * 说明：按虚功关系 tau=J^T*f；空指针不操作，支持 tau 与 f 共用数组。
 */
void vmc_torque(const float J[2][2], const float f[2], float tau[2]);

/* vmc_foot_vel：由电机角速度计算足端速度
 * 参数：J 为位置雅可比；qd 为两个电机角速度，单位 rad/s；v 接收 {vx, vz}
 * 返回：无
 * 说明：v=J*qd，单位 m/s；空指针不操作，支持 v 与 qd 共用数组。
 */
void vmc_foot_vel(const float J[2][2], const float qd[2], float v[2]);

#endif
