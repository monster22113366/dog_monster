/*
 * 本文件属于 App 应用层，组合 Bsp IMU 与 att_est 组件实现机身姿态任务。
 * 依赖 bsp_imu 的硬件接口、att_est 卡尔曼、app_ctrl 控制时基及标准数学/内存函数。
 * 安装方向由 IMU_MOUNT 把芯片系换算到机体系；结果集中在 g_imu。
 */
#include "app_imu.h"
#include "bsp_imu.h"
#include "att_est.h"
#include "app_ctrl.h"
#include <math.h>
#include <string.h>

#ifndef APP_IMU_TIMEOUT_MS
/* 无新样本超过该时长判为失效，单位毫秒。 */
#define APP_IMU_TIMEOUT_MS 20u
#endif

/* IMU_MOUNT为芯片系 -> 机体系映射矩阵（行优先）。
 * USB 侧朝狗头，为+X
 * CAN1/CAN2 侧朝狗左，为+Y
 * 板子带stm32的那一面朝天空，为+Z
 此方向的芯片系即机体系；若安装改变，按此修改矩阵。 */
static const float IMU_MOUNT[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

/* 卡尔曼参数取 SIL 验证值；顺序：q_ang, q_bias, r, acc_gate。 */
static att_cfg_t s_cfg = {1e-6f, 1e-8f, 0.03f, 0.0f};

volatile imu_debug_t g_imu;

/* 非 volatile 记录快照，仅在本模块初始化和 tick 末尾更新。 */
static struct {
    float roll, pitch;
    uint8_t init_err, ok;
} s_imu_log;
static const sim_var_desc_t s_imu_vars[] = {
    {"roll", SIM_VAR_F32, 0u, &s_imu_log.roll},
    {"pitch", SIM_VAR_F32, 0u, &s_imu_log.pitch},
    {"init_err", SIM_VAR_U8, 0u, &s_imu_log.init_err},
    {"ok", SIM_VAR_U8, 0u, &s_imu_log.ok},
};
const sim_vars_t g_imu_vars = {"imu", s_imu_vars, sizeof(s_imu_vars) / sizeof(s_imu_vars[0])};

static struct {
    att_state_t att;
    uint32_t last_ms;      /* 上一次调用的控制节拍 ms */
    uint32_t last_ok_ms;   /* 上一次收到有效样本的计时 */
} s_imu;

/* log_snapshot：逐项读取 volatile 状态，发布记录器使用的普通存储快照。 */
static void log_snapshot(void) {
    s_imu_log.roll = g_imu.roll;
    s_imu_log.pitch = g_imu.pitch;
    s_imu_log.init_err = (uint8_t)g_imu.init_err;
    s_imu_log.ok = g_imu.ok;
}

/* app_imu_init：复位调试状态、初始化硬件与滤波器
 * 参数：无
 * 返回：无，Bsp 返回值写入 g_imu.init_err
 * 说明：默认测量噪声 0.03；在硬件初始化结束后记录时间，避免启动延时计入首拍。
 */
void app_imu_init(void) {
    memset((void *)&g_imu, 0, sizeof(g_imu));
    memset(&s_imu, 0, sizeof(s_imu));
    g_imu.kf_r = 0.03f;
    s_cfg.r = g_imu.kf_r;
    g_imu.init_err = (int8_t)bsp_imu_init();
    att_init(&s_imu.att, &s_cfg);
    /* 控制时基在拍末推进；首拍同样按完整的 2 ms 积分。 */
    s_imu.last_ms = app_ctrl_ms() - APP_CTRL_PERIOD_MS;
    s_imu.last_ok_ms = app_ctrl_ms();
    log_snapshot();
}

/* app_imu_tick：读取最新样本并发布机身姿态
 * 参数：无
 * 返回：无
 * 说明：初始化失败只保持 ok=0；正值 kf_r 每拍同步，超时采用可回绕的毫秒差。
 */
void app_imu_tick(void) {
    if (g_imu.init_err != 0) {
        g_imu.ok = 0u;
        log_snapshot();
        return;
    }
    const uint32_t now = app_ctrl_ms();
    float dt = (uint32_t)(now - s_imu.last_ms) * 0.001f;
    s_imu.last_ms = now;
    dt = dt < 0.001f ? 0.001f : (dt > 0.02f ? 0.02f : dt);

    const float r = g_imu.kf_r;
    if (isfinite(r) && r > 0.0f)
        s_cfg.r = r;

    bsp_imu_sample_t raw;
    if (bsp_imu_read(&raw) == 0) {
        float gyro[3], acc[3];
        for (int i = 0; i < 3; i++) {
            gyro[i] = IMU_MOUNT[i][0] * raw.gyro[0] + IMU_MOUNT[i][1] * raw.gyro[1] + IMU_MOUNT[i][2] * raw.gyro[2];
            acc[i] = IMU_MOUNT[i][0] * raw.acc[0] + IMU_MOUNT[i][1] * raw.acc[1] + IMU_MOUNT[i][2] * raw.acc[2];
        }
        att_update(&s_imu.att, &s_cfg, gyro, acc, dt);
        for (int i = 0; i < 3; i++) {
            g_imu.gyro[i] = gyro[i] - s_imu.att.bias[i];
            g_imu.gyro_raw[i] = gyro[i];
            g_imu.acc[i] = acc[i];
        }
        g_imu.roll = s_imu.att.roll;
        g_imu.pitch = s_imu.att.pitch;
        g_imu.count++;
        s_imu.last_ok_ms = now;
    }
    g_imu.ok = (uint32_t)(now - s_imu.last_ok_ms) <= APP_IMU_TIMEOUT_MS && g_imu.count > 1u;
    log_snapshot();
}
