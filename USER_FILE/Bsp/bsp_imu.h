/*
 * 本文件属于 Bsp 硬件层，定义板载 IMU（达妙 MC02 的 BMI088，SPI2，加速度计片选 PC0、陀螺仪片选 PC3）的读取接口。
 * 输出芯片坐标系样本：gyro 角速度 rad/s，acc 比力 m/s²；量程、ODR 与坐标轴见 bsp_imu.c。
 * 由 app_imu 调用初始化和周期读取；本头文件仅依赖 <stdint.h>，实现依赖 STM32 HAL SPI。
 * 仿真中由 dog_matlabsim/csrc/sim_bsp_imu.c 替代实现。
 */
#ifndef BSP_IMU_H
#define BSP_IMU_H
#include <stdint.h>

typedef struct {
    float gyro[3]; /* 芯片系角速度 rad/s */
    float acc[3];  /* 芯片系比力 m/s² */
} bsp_imu_sample_t;

/* bsp_imu_init：配置 BMI088 量程与 ODR
 * 参数：无
 * 返回：0 成功；负数为初始化步骤错误，具体对应见实现中的返回点。
 * 说明：在 GPIO、SPI2 初始化之后调用；包含阻塞延时，不在周期任务中调用。
 */
int bsp_imu_init(void);

/* bsp_imu_read：读取一次最新样本
 * 参数：out 接收样本
 * 返回：0 成功；1 未初始化成功；负数为参数或通信错误。
 * 说明：每次调用均读取两颗传感器，失败时 out 不变。
 */
int bsp_imu_read(bsp_imu_sample_t *out);

#endif
