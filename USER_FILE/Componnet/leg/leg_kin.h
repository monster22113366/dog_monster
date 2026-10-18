/*
 * 本文件属于 Componnet 算法组件层，定义同轴双髋电机腿的平面运动学。
 * 髋部为原点，+x 向前，+z 向下；长度单位 m，角度单位 rad，从 +z 向 +x 为正。
 * 两电机角为 qh、qk，足端射线角 gamma=(qh+qk)/2，半张角 psi=(qk-qh)/2。
 * 大腿长 l1、小腿长 l2；取余弦定理的正根分支：
 * l=l1*cos(psi)+sqrt(l2*l2-l1*l1*sin(psi)*sin(psi))，
 * x=l*sin(gamma)，z=l*cos(gamma)。逆解取 psi=acos(c)，范围 [0, pi]。
 * 头文件无外部依赖；实现仅依赖 <math.h>，无 HAL 依赖及可变全局状态。
 */
#ifndef COMPONENT_LEG_KIN_H
#define COMPONENT_LEG_KIN_H

typedef struct {
    float l1, l2, l_min, l_max; /* 杆长及允许腿长范围，单位 m */
} leg_geom_t;

/* leg_fk：由双电机角度求足端位置
 * 参数：g 为几何参数；qh、qk 为电机角；p 接收 {x, z}，单位 m
 * 返回：成功为 0；任一指针为空或根号内为负时为 -1
 * 说明：失败时不改写 p；正解不施加 l_min、l_max 工作范围限制。
 */
int leg_fk(const leg_geom_t *g, float qh, float qk, float p[2]);

/* leg_ik：由足端位置求双电机角度
 * 参数：g 为几何参数；p 为 {x, z}；qh、qk 接收电机角，单位 rad
 * 返回：成功为 0；空指针、l1<=0、腿长非正或超出允许范围时为 -1
 * 说明：调用者提供与正根分支一致的工作范围；余弦值截取到 [-1, 1]。
 */
int leg_ik(const leg_geom_t *g, const float p[2], float *qh, float *qk);

/* leg_jacobian：计算足端位置相对电机角的解析雅可比
 * 参数：g 为几何参数；qh、qk 为电机角；J 接收 2x2 矩阵，单位 m/rad
 * 返回：无
 * 说明：行依次为 x、z，列依次为 qh、qk；空指针不操作，根项非正时用 1e-6。
 */
void leg_jacobian(const leg_geom_t *g, float qh, float qk, float J[2][2]);

#endif
