/*
 * 本文件属于 App 应用层，供调试使用。
 * 按固定抽取周期把一组 Fast 变量记录到 AXI SRAM，PC 端用 OpenOCD dump 后转 CSV；不需要时把记录器从控制链移除即可。
 * 记录内容用「描述符表 + void 指针」组织：每个模块把自己的变量以 sim_vars_t 描述，app_log 只管按类型搬运，
 * 新增被记录量只改描述符表，不改本文件的搬运逻辑。
 * 每帧（T=12 ms）写成固定长度：帧头 8 字节（count u16、时间戳 u32、flags u16）+ 各变量原始值 + 帧尾 16 位校验（对前序字节逐字节异或）。
 * 依赖各模块导出的 g_* 描述符及 app_ctrl 控制节拍 ms；缓冲区不进入链接脚本，也不由启动代码清零。
 * 帧时间戳为 app_ctrl_ms() 的完整 32 位，单位是控制节拍 ms。
 * flags 位 0/1/2 依次表示 IMU 有效、整机故障、全部电机 RUN；ctrl 字段为上一拍统计。
 */
#ifndef APP_LOG_H
#define APP_LOG_H

#include <stddef.h>
#include <stdint.h>

#ifndef APP_LOG_BASE
/* AXI SRAM 起始地址，链接脚本未使用该区，且复位不清零（见 STM32H723XG_FLASH.ld）。 */
#define APP_LOG_BASE 0x24000000u
#endif

#ifndef APP_LOG_CAPACITY
/* 记录缓冲区总字节数。 */
#define APP_LOG_CAPACITY (128u * 1024u)
#endif

/* 变量类型：全部按原始存储记录，PC 端按此解析，保证无精度损失。 */
typedef enum {
    SIM_VAR_U8 = 0, SIM_VAR_I8, SIM_VAR_U16, SIM_VAR_I16, SIM_VAR_U32, SIM_VAR_F32
} sim_var_type_t;

typedef struct {
    const char *name;  /* 变量名，供 PC 端与人工核对 */
    uint8_t type;      /* sim_var_type_t */
    uint16_t extra;    /* 数组长度；0 表示单个标量（单个数组时 same as 1） */
    const void *addr;  /* 非 volatile 快照地址，在整个记录期间有效；按字节搬运，无额外对齐要求 */
} sim_var_desc_t;

typedef struct {
    const char *name;   /* 模块名，作为 PC 端 CSV 的前缀 */
    const sim_var_desc_t *vars;
    uint16_t n;         /* 变量条目数 */
} sim_vars_t;

/* 帧头 8 字节 + 数据区（由描述符表决定）+ 校验 2 字节 */
#define APP_LOG_FRAME_OVERHEAD 10u

/* 缓冲区头：固定 32 字节，PC 端解析用。 */
typedef struct __attribute__((aligned(8))) {
    uint32_t magic;        /* 0x4C4F4F50，'LOOP' */
    uint16_t version;      /* 格式版本 1 */
    uint16_t frame_ms;     /* 每帧周期，单位为控制节拍 ms */
    uint32_t capacity;     /* 缓冲区总字节数 */
    uint32_t frame_bytes;  /* 每帧字节数 */
    uint32_t frame_max;    /* 最多帧数 */
    uint32_t count;        /* 已写帧数，每帧更新 */
    uint32_t reserved;     /* 保留，使头长度为 32 字节 */
} app_log_hdr_t;

_Static_assert(sizeof(app_log_hdr_t) == 32u, "app_log_hdr_t 必须为 32 字节");
_Static_assert(offsetof(app_log_hdr_t, count) == 20u, "app_log_hdr_t.count 偏移固定");

/* 调试接口：start 为输入，其余由记录器输出。 */
typedef struct {
    volatile uint8_t start;   /* 输入：写 1 请求新一轮记录（覆盖旧数据），处理后自动清 0 */
    volatile uint8_t state;   /* 输出：0 空闲 / 1 记录中 / 2 已满 */
    volatile uint8_t enabled; /* 输出：1 = 本轮记录中 */
    volatile uint32_t count;  /* 输出：已写帧数 */
    volatile float seconds;   /* 输出：已记录时长 s */
} app_log_debug_t;

extern volatile app_log_debug_t g_log;

/* app_log_init：登记描述符表并清空调试接口
 * 说明：不主动开始记录，等待 g_log.start=1（或调用 app_log_start）。
 */
void app_log_init(void);

/* app_log_start：开始一轮记录
 * 说明：在控制中断或定时器启动前调用；主循环运行时通过 g_log.start 请求，避免并发写头。
 * 写头并复位帧计数，之后每 APP_LOG_FRAME_MS 由控制链调用 app_log_tick 写一帧。
 */
void app_log_start(void);

/* app_log_tick：按帧周期搬运一帧数据
 * 说明：按控制拍数分频到 APP_LOG_FRAME_MS；时间戳取控制节拍 ms，未开始记录时立即返回。
 */
void app_log_tick(void);

/* app_log_frame_ms：当前帧周期
 */
uint16_t app_log_frame_ms(void);

#endif
