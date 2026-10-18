/**
 * 本文件属于 App 应用层，定义 ET07 / RF207S（W.BUS 兼容 SBUS）遥控接口。
 * 依赖 app_log.h 的描述符类型；组合 Bsp SBUS 与纯算法 sbus 组件。
 * 由 app_ctrl 每 10 ms 在 TIM6 中断中调用，排在 app_legs 之前。
 *
 * 遥控器操作说明：
 * - 左侧区域控制腿部：
 *   SA：零位校准；
 *   SB：切换腿部模式（PASSIVE / STAND / TROT）；
 *   左摇杆：调节当前腿部模式对应的参数。
 *
 * - 右侧区域控制底盘：
 *   SD：急停；
 *   SC：切换底盘模式（OFF / HOLD / DRIVE）；
 *   右摇杆：提供当前底盘模式对应的运动参数。
 *
 * 遥控使能：
 * APP_RC_ENABLE_DEFAULT=0：上电默认仅监视遥控数据，不执行遥控控制；
 * APP_RC_ENABLE_DEFAULT=1：上电默认使能遥控控制。
 * 脱离调试器实际使用时，应将 APP_RC_ENABLE_DEFAULT 设为 1。
 */ 
#ifndef APP_RC_H
#define APP_RC_H

#include "app_log.h"

#ifndef APP_RC_ENABLE_DEFAULT
/* 遥控控制默认关闭；调试确认通道后可覆盖为 1。 */
#define APP_RC_ENABLE_DEFAULT 1u
#endif

#if APP_RC_ENABLE_DEFAULT
#pragma message("APP_RC_ENABLE_DEFAULT=1: 遥控控制默认使能。")
#endif
/* ET07 默认映射，下标从 0 起：CH2 右摇杆纵向、CH5 SB 三挡、CH9 SD 瞬时。 */
#ifndef APP_RC_CH_FWD
#define APP_RC_CH_FWD 1u /* CH2 右摇杆纵向，轮前进后退。 */
#endif
#ifndef APP_RC_CH_MODE
#define APP_RC_CH_MODE 4u
#endif
#ifndef APP_RC_CH_WHEEL
#define APP_RC_CH_WHEEL 7u /* CH8 SC 三挡，低/中/高 = OFF/HOLD/DRIVE。 */
#endif
#ifndef APP_RC_CH_YAW
#define APP_RC_CH_YAW 0u /* CH1 右摇杆横向。 */
#endif
#ifndef APP_RC_CH_LEG
#define APP_RC_CH_LEG 2u /* CH3 左摇杆纵向，油门式，STAND 站高 / TROT 步长。 */
#endif
#ifndef APP_RC_CH_ESTOP
#define APP_RC_CH_ESTOP 8u
#endif
#ifndef APP_RC_CH_ZERO
#define APP_RC_CH_ZERO 9u
#endif
#ifndef APP_RC_CH_RESET
#define APP_RC_CH_RESET 5u /* ET07 CH6 旋钮，下标从 0 起。 */
#endif
#ifndef APP_RC_RESET_LEFT
#define APP_RC_RESET_LEFT 400u /* 实测左极限 361，保留余量。 */
#endif
#ifndef APP_RC_RESET_RIGHT
#define APP_RC_RESET_RIGHT 1660u /* 实测右极限 1703，保留近似对称余量。 */
#endif
#ifndef APP_RC_RESET_WINDOW_MS
#define APP_RC_RESET_WINDOW_MS 3000u /* 第一次到最右至第二次到最右的总时间上限。 */
#endif
#ifndef APP_RC_LOST_MS
/* lost 连续超过该时间视为失联，单帧/零星丢帧不触发；接收机 failsafe
 * 要 0.7~1 s 才出现，不能单靠它判断失联。 */
#define APP_RC_LOST_MS 100u
#endif
#ifndef APP_RC_LOSS_RELEASE_MS
/* 单位 ms：失联处理先置 HOLD（按斜坡减速并零速阻尼保持），经过此时间仍未恢复
 * 则请求 OFF，底盘随之失能四轮，电机 phase 回 IDLE。保持时间可由用户调整。
 * 0：失联当拍直接失能滑行，无减速斜坡，不建议；0xFFFFFFFFu：一直保持不放开。
 * 注意：放开后车体在斜坡上可能溜车。 */
#define APP_RC_LOSS_RELEASE_MS 2000u
#endif
#ifndef APP_RC_STRIDE_MAX
/* 最大步长 m，首次实机建议下调，仿真验证过 0.04。 */
#define APP_RC_STRIDE_MAX 0.06f
#endif
#ifndef APP_RC_STRIDE_ZERO_NORM
/* 左摇杆拉到该位置以下视为拉到底(步长 0 并武装),抵消不同遥控器行程的零点偏差 */
#define APP_RC_STRIDE_ZERO_NORM -0.9f
#endif
#ifndef APP_RC_DEADBAND
#define APP_RC_DEADBAND 0.12f
#endif
#ifndef APP_RC_TIMEOUT_MS
#define APP_RC_TIMEOUT_MS 100u
#endif
#ifndef APP_RC_SWITCH_STABLE
/* 三挡开关需要连续收到的相同档位帧数，不按 tick 重复计数。 */
#define APP_RC_SWITCH_STABLE 3u
#endif
#ifndef APP_RC_RETRY_MS
/* TROT 请求在 app_legs 渐变门控期间的重发间隔，单位毫秒。 */
#define APP_RC_RETRY_MS 100u
#endif

#ifndef APP_RC_ZERO_HOLD_MS
/* SA 新档位需连续保持的时长，单位毫秒。 */
#define APP_RC_ZERO_HOLD_MS 1000u
#endif
#ifndef APP_RC_ESTOP_ON
#define APP_RC_ESTOP_ON 0.5f
#endif
#ifndef APP_RC_ESTOP_OFF
#define APP_RC_ESTOP_OFF 0.2f
#endif

typedef struct {
    volatile uint8_t enable;   /* 输入：1 = 遥控器控制整机（写 g_legs / g_chassis / 急停），0 = 只监视（关闭沿释放自有底盘一次，之后不写控制量）；默认 0 */
    int8_t init_err;            /* 输出：bsp_sbus_init 返回值。 */
    uint8_t online;             /* 输出：当前 fresh。 */
    uint8_t ever_online;        /* 输出：曾经 fresh，失控保护的启用条件。 */
    uint32_t frames;            /* 输出：本任务消费的有效帧数。 */
    uint32_t lost_count;        /* 输出：在线后丢失信号次数；每次失控保护只计一次。 */
    volatile uint32_t frame_lost; /* 输出：接收机报告的单帧丢失次数（f[23] bit2），只统计。 */
    uint32_t bad_count;         /* 输出：Bsp 坏帧数。 */
    uint32_t err_count;         /* 输出：Bsp UART 错误数。 */
    uint32_t age_ms;            /* 输出：距有效帧的毫秒数；从未收到为 UINT32_MAX。 */
    uint32_t good_age_ms;       /* 输出：距非 lost、非 failsafe 好帧的毫秒数；从未收到为 UINT32_MAX。 */
    uint32_t reset_count;       /* 输出：检测到完整复位手势的次数，监视模式也计数。 */
    uint16_t ch[16];            /* 输出：最近有效帧的原始通道值。 */
    uint8_t sw_legs;            /* 输出：SB 当前原始档位，低/中/高 = 0/1/2。 */
    uint8_t sw_wheel;           /* 输出：SC 当前原始档位，低/中/高 = 0/1/2。 */
    uint8_t sw_zero;            /* 输出：SA 当前档位 0/1。 */
    uint8_t estop;              /* 输出：CH9 按下状态，按 ON/OFF 阈值滞回。 */
    float stride_cmd;          /* 输出：小跑重新武装后的步长命令 m，离线为 0。 */
    uint32_t zero_count;        /* 输出：已下发零位请求次数。 */
    uint32_t zero_reject;       /* 输出：因前置条件不满足而作废的拨动次数。 */
} rc_debug_t;

extern volatile rc_debug_t g_rc;
extern const sim_vars_t g_rc_vars;

/* app_rc_init：在电机初始化后、控制定时器启动前复位遥控状态并启动接收。 */
void app_rc_init(void);

/* app_rc_tick：消费最新帧，更新模式、步长、急停及一次性失控保护。 */
void app_rc_tick(void);

#endif
