/*
 * 本文件属于 App 应用层，定义姿态任务：读取 Bsp IMU 样本、换算到机体系、调用 att_est 卡尔曼并发布 g_imu。
 * 机体系 +x 前、+y 左、+z 上；roll 左侧抬起为正、pitch 机头向下为正，单位 rad。
 * 依赖 <stdint.h> 与 app_log.h 描述符；app_ctrl 每 2 ms 调用任务，app_legs 只读 g_imu。
 */
#ifndef APP_IMU_H
#define APP_IMU_H
#include "app_log.h"
#include <stdint.h>

#ifndef APP_IMU_PERIOD_MS
/* 姿态任务调度周期，单位毫秒。 */
#define APP_IMU_PERIOD_MS 2u
#endif

typedef struct {  /* 调试接口：输入由调试器写，输出由 app_imu 写 */
    volatile int8_t init_err;    /* 输出：bsp_imu_init 返回值，0 正常，负数对应初始化步骤 */
    volatile float kf_r;         /* 输入：KF 倾角测量噪声 rad²，默认 0.03；仅接受有限正值 */
    volatile float roll, pitch;  /* 输出：姿态 rad */
    volatile float gyro[3];      /* 输出：机体系角速度 rad/s（已减去零偏估计） */
    volatile float gyro_raw[3];  /* 输出：机体系角速度 rad/s（安装旋转后、未减零偏），供记录器使用 */
    volatile float acc[3];       /* 输出：机体系比力 m/s² */
    volatile uint8_t ok;         /* 输出：1 = 最近 APP_IMU_TIMEOUT_MS 内收到有效样本且滤波已初始化 */
    volatile uint32_t count;     /* 输出：已处理样本数 */
} imu_debug_t;

extern volatile imu_debug_t g_imu;
extern const sim_vars_t g_imu_vars;

/* app_imu_init：初始化板载 IMU 并复位卡尔曼状态
 * 参数：无
 * 返回：无，初始化结果见 g_imu.init_err
 * 说明：在 GPIO、SPI2 初始化后、app_motorSHS_init 前调用一次，包含硬件启动延时。
 */
void app_imu_init(void);

/* app_imu_tick：取样本、换算、滤波并发布
 * 参数：无
 * 返回：无
 * 说明：每 APP_IMU_PERIOD_MS 调用；初始化失败或样本超时时 ok 清 0，关闭姿态修正。
 */
void app_imu_tick(void);

#endif
