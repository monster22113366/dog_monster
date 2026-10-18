/*
 * 本文件属于 Componnet 组件层，实现 MW 电调 CAN 协议的帧编解码与各节点状态表。
 * 协议为 ODrive CANSimple 子集，不支持 0x008 MIT 和输入模式 9。
 * 对外提供注册发送回调、按 cmd 组帧发送、接收解析、状态查询及各命令的 inline 封装。
 * 启动代码以 mwSHS_init(&ops) 注册发送函数，并为总线登记 mwSHS_recv；
 * App 层调用 mwSHS_xxx 下发命令，通过 mwSHS_snapshot 读取一致快照。
 * CAN ID = node(6 bit) << 5 | cmd(5 bit)，共 11 位；数据固定 8 字节、小端。
 * 假设 MCU 为小端、float 为 32 位 IEEE-754；除 0x008 MIT 外，位置/速度/力矩均为电机转子侧
 * （减速前），单位 turns、turns/s、Nm（手册 0x08 节：仅 CAN MIT 为输出轴侧）；电流 A，电压 V。
 * 关节力矩与转子力矩的换算见 app_legs.c 的 LEGS_TORQUE_RATIO。
 * 依赖标准头文件；无平台硬件依赖；内部状态表为文件静态变量，
 * 中断更新缓存，mwSHS_snapshot 使用平台短临界区保护多字段读取。
 * 使用示例（分别用于初始化、接收中断及主循环；读取状态时需保护）：
 * mwSHS_init(&ops);
 * mwSHS_recv(bus, &frame);
 * mwSHS_torque(0, 1, 0.1f);
 * mwSHS_motor_t m; mwSHS_snapshot(0, 1, &m);
 */
#ifndef MW_CANSHS_H
#define MW_CANSHS_H

#include <stdint.h>
#include <stdint.h>
#include <stddef.h>

/* 独立协议帧；适配层逐字段转换，不依赖平台布局。 */
typedef struct {
    uint32_t id;
    uint8_t len;
    uint8_t data[64];
    uint8_t extended, fd, brs;
} mwSHS_frame_t;
typedef int (*mwSHS_tx_fn)(uint8_t bus, const mwSHS_frame_t *frame);
typedef uint32_t (*mwSHS_lock_fn)(void);
typedef void (*mwSHS_unlock_fn)(uint32_t key);
typedef struct {
    mwSHS_tx_fn tx;
    mwSHS_lock_fn lock;
    mwSHS_unlock_fn unlock;
} mwSHS_ops_t;

#include <string.h>
/* 总线数，即状态表行数；可在编译选项覆盖，本工程 CMakeLists.txt 设为 2。 */
#ifndef MWSHS_BUS_NUM
#define MWSHS_BUS_NUM 4
#endif
/* 节点号上限（不含），即状态表列数；协议允许节点号 0~63，可在编译选项覆盖。 */
#ifndef MWSHS_NODE_NUM
#define MWSHS_NODE_NUM 16
#endif
/* 编译期检查表维度：总线数 1~255，节点数 1~64，避免越出索引类型或协议范围。 */
#if MWSHS_BUS_NUM < 1 || MWSHS_BUS_NUM > 255 || MWSHS_NODE_NUM < 1 || MWSHS_NODE_NUM > 64
#error "MW_BUS_NUM must be 1..255; MW_NODE_NUM must be 1..64"
#endif

/* 枚举值对应 CAN ID 的 5 位 cmd 字段；下行为主控发送，请求-应答使用同一 ID。 */
typedef enum {
    MWSHS_CMD_HEARTBEAT=0x01,    /* 心跳，电调周期上报。 */
    MWSHS_CMD_ESTOP=0x02,        /* 急停，下行；需清错误才能恢复。 */
    MWSHS_CMD_ERROR=0x03,        /* 查询错误，请求-应答；请求 B0 为错误类型。 */
    MWSHS_CMD_RXSDO=0x04,        /* SDO 访问，下行；B0 为 0 读/1 写，B1~B2 端点号 u16，B4~B7 值；读结果由 0x005 回复，不解析。 */
    MWSHS_CMD_TXSDO=0x05,        /* SDO 回复，上行；应答 0x004 读请求，格式同 0x004；不解析。 */
    MWSHS_CMD_NODE_ID=0x06,      /* 设节点号，下行；B0~B3 u32；本工程用上位机设置。 */
    MWSHS_CMD_STATE=0x07,        /* 设轴状态，下行。 */
    MWSHS_CMD_ENCODER=0x09,      /* 编码器 pos/vel，请求-应答；转子侧 turns、turns/s。 */
    MWSHS_CMD_COUNT=0x0a,        /* 多圈/单圈编码器计数，请求-应答；counts。 */
    MWSHS_CMD_MODE=0x0b,         /* 设控制模式及输入模式，下行。 */
    MWSHS_CMD_POS=0x0c,          /* 位置及前馈，下行；turns、0.001 turns/s、0.001 Nm。 */
    MWSHS_CMD_VEL=0x0d,          /* 速度及力矩前馈，下行；turns/s、Nm。 */
    MWSHS_CMD_TORQUE=0x0e,       /* 力矩指令，下行；转子侧 Nm。 */
    MWSHS_CMD_LIMITS=0x0f,       /* 限速 turns/s、限流 A，下行。 */
    MWSHS_CMD_ANTICOGGING=0x10,  /* 齿槽校准，下行；电机会转动。 */
    MWSHS_CMD_TRAJ_VEL=0x11,     /* 梯形轨迹限速，下行；turns/s。 */
    MWSHS_CMD_TRAJ_ACCEL=0x12,   /* 梯形轨迹加减速，下行；turns/s²。 */
    MWSHS_CMD_TRAJ_INERTIA=0x13, /* 梯形轨迹惯量，下行；Nm/(turns/s²)。 */
    MWSHS_CMD_IQ=0x14,          /* 电流设定/实测，请求-应答；A。 */
    MWSHS_CMD_REBOOT=0x16,      /* 重启，下行。 */
    MWSHS_CMD_BUS=0x17,         /* 母线电压 V、电流 A，请求-应答。 */
    MWSHS_CMD_CLEAR_ERRORS=0x18, /* 清错误，下行。 */
    MWSHS_CMD_MOVE_INC=0x19,     /* 增量移动，下行；B0~B3 float 转子侧位移 turns。 */
    MWSHS_CMD_POS_GAIN=0x1a,    /* 位置环增益，下行；(turns/s)/turn。 */
    MWSHS_CMD_VEL_GAIN=0x1b,    /* 速度环比例/积分增益，下行；Nm/(turns/s)、Nm/turn。 */
    MWSHS_CMD_TORQUES=0x1c,     /* 力矩设定/实测，请求-应答；Nm。 */
    MWSHS_CMD_POWERS=0x1d,      /* 电功率/机械功率，请求-应答；W。 */
    MWSHS_CMD_DISABLE_CAN=0x1e, /* 禁用 CAN 并重启，下行；恢复需借助上位机，本工程不使用。 */
    MWSHS_CMD_SAVE=0x1f         /* 保存配置并重启，下行。 */
} mwSHS_cmd_t;

/* 0x007 目标轴状态 / 0x001 心跳 B4 当前轴状态；常用 1 IDLE 关 PWM、8 CLOSED_LOOP 闭环。
 * 值 5 未定义；手册 0x007 只列 0/1/3/4/7/8，其余沿用 ODrive；状态切换是否成功需由心跳确认。
 */
typedef enum {
    MWSHS_STATE_UNDEFINED=0,     /* 未定义。 */
    MWSHS_STATE_IDLE=1,          /* 空闲，关闭 PWM。 */
    MWSHS_STATE_STARTUP=2,       /* 启动序列。 */
    MWSHS_STATE_FULL_CALIB=3,    /* 电机与编码器完整校准。 */
    MWSHS_STATE_MOTOR_CALIB=4,   /* 测相电阻、相电感。 */
    MWSHS_STATE_INDEX_SEARCH=6,  /* 寻找编码器 index。 */
    MWSHS_STATE_OFFSET_CALIB=7,  /* 编码器偏移校准。 */
    MWSHS_STATE_CLOSED_LOOP=8,   /* 闭环控制。 */
    MWSHS_STATE_LOCKIN=9,        /* 锁定自转，需已校准。 */
    MWSHS_STATE_DIR_FIND=10,     /* 编码器方向搜索，需已校准。 */
    MWSHS_STATE_HOMING=11,       /* 回零。 */
    MWSHS_STATE_HALL_POLARITY=12, /* 霍尔极性校准。 */
    MWSHS_STATE_HALL_PHASE=13,   /* 霍尔相位校准。 */
    MWSHS_STATE_ANTICOGGING=14   /* 齿槽转矩校准。 */
} mwSHS_state_t;

/* 0x00B 的首个参数（B0~B3）：控制模式决定指令含义，输入模式决定指令如何生效。
 * 参考工程用 3+5（位置+梯形轨迹），本工程力矩模式用 1+1（力矩+直接）。
 * 手册 Modbus 0x65 节：速度斜坡仅用于速度控制，位置滤波仅用于位置控制，力矩斜坡仅用于力矩控制；
 * 电压模式本工程不使用。
 */
typedef enum { MWSHS_CTRL_VOLTAGE=0, MWSHS_CTRL_TORQUE=1, MWSHS_CTRL_VEL=2, MWSHS_CTRL_POS=3 } mwSHS_control_t;
/* 0x00B 的第二个参数（B4~B7）：输入模式；不支持输入模式 9。斜坡/滤波参数经 0x004 SDO 写入，端点号随固件版本不同，查对应 endpoint JSON。 */
typedef enum {
    MWSHS_INPUT_IDLE=0,        /* 不接受输入。 */
    MWSHS_INPUT_DIRECT=1,      /* 直接输入。 */
    MWSHS_INPUT_VEL_RAMP=2,    /* 速度斜坡，斜率端点 vel_ramp_rate。 */
    MWSHS_INPUT_POS_FILTER=3,  /* 位置滤波，带宽端点 input_filter_bandwidth。 */
    MWSHS_INPUT_TRAP=5,        /* 梯形轨迹。 */
    MWSHS_INPUT_TORQUE_RAMP=6  /* 力矩斜坡，斜率端点 torque_ramp_rate。 */
} mwSHS_input_t;

/* 节点状态缓存；请求-应答型字段保存最近一次回包，发出请求后不会立即更新。 */
typedef struct {
    uint32_t axis_error;    /* 0x001 B0~B3 轴错误码，非 0 表示故障。 */
    uint8_t state,         /* 0x001 B4 当前轴状态，取值见 mwSHS_state_t。 */
            motor_err,     /* 0x001 B5 bit0 电机错误标志。 */
            encoder_err,   /* 0x001 B5 bit1 编码器错误标志。 */
            controller_err, /* 0x001 B5 bit2 控制器错误标志。 */
            system_err,    /* 0x001 B5 bit3 系统错误标志。 */
            traj_done,     /* 0x001 B5 bit7 梯形轨迹完成标志。 */
            life;          /* 0x001 B7 心跳计数，每帧加 1，0~255 回绕；不连续表示丢帧。 */
    uint64_t error;         /* 0x003 回复错误码 B0~B7；请求 B0 类型 0 电机(u64)/1 编码器/3 控制/4 系统(u32，高 4 字节为 0)。
                           * 位定义见手册 CANopen 故障码表 ODrive 列。
                           */
    float pos,             /* 0x009 转子侧位置，turns。 */
          vel;             /* 0x009 转子侧速度，turns/s。 */
    int32_t shadow_count,  /* 0x00A 多圈编码器计数，counts。 */
            count_in_cpr;  /* 0x00A 单圈编码器计数，counts。 */
    float iq_set,          /* 0x014 设定电流，A。 */
          iq_meas,         /* 0x014 实测电流，A。 */
          bus_v,           /* 0x017 母线电压，V。 */
          bus_i,           /* 0x017 母线电流，A。 */
          torque_set,      /* 0x01C 设定力矩，Nm。 */
          torque,          /* 0x01C 实测力矩，Nm。 */
          elec_power,      /* 0x01D 电功率，W。 */
          mech_power;      /* 0x01D 机械功率，W。 */
    uint32_t rx_count;     /* 成功解析帧数，所有 cmd 共用；初始化后只增不减，可回绕。
                           * 用于在线检测，一段时间不变即判定离线。
                           */
} mwSHS_motor_t;

/* 发送回调消费通用硬件帧，不得保留栈指针。
 * 返回 1 入队、0 FIFO 满、-1 错误，与 平台发送函数 一致。
 * 标准/扩展格式由本组件学习并选择，BSP 不推断协议。 */


/* mwSHS_init：清空状态表并保存平台能力的静态副本
 * 参数：ops 为发送和临界区能力，可为 NULL
 * 返回：无
 * 说明：tx 缺失时不发送；lock/unlock 任一缺失时不进入临界区。
 */
void mwSHS_init(const mwSHS_ops_t *ops);

/* mwSHS_get：返回节点状态表项的只读指针
 * 参数：bus 为总线号；node 为节点号
 * 返回：内部表项指针；bus/node 越界返回 NULL
 * 说明：中断可能同时写入，读取需由调用方保护，使用 mwSHS_snapshot。
 */
const mwSHS_motor_t *mwSHS_get(uint8_t bus, uint8_t node);
/* 一致快照：成功为 1；非法索引清零输出并返回 0，NULL 输出返回 0。 */
int mwSHS_snapshot(uint8_t bus, uint8_t node, mwSHS_motor_t *out);

/* mwSHS_send：组 8 字节帧并调用发送回调
 * 参数：bus 为总线号；node 为节点号；cmd 为命令；payload 为载荷；len 为载荷字节数
 * 返回：无
 * 说明：payload 可为 NULL（全 0）；len 超过 8 截断，不足补 0。
 *       回调未注册、bus/node 越界或 cmd>31 时直接返回。
 */
void mwSHS_send(uint8_t bus, uint8_t node, uint8_t cmd, const void *payload, uint8_t len);

/* mwSHS_recv：接收 BSP 投递的八字节 MW 帧，解析后增加 rx_count。
 * 拒绝无效总线、节点、ID、长度和未知命令；心跳/编码器学习帧格式。
 * 在 RX 中断调用；控制任务用 mwSHS_snapshot 读取多字段状态。 */
void mwSHS_recv(uint8_t bus, const mwSHS_frame_t *frame);

/* 以下封装的 b 为总线号，n 为节点号，均交由 mwSHS_send 组帧发送。 */

/* mwSHS_estop：发送 0x002 急停
 * 参数：b 为总线号；n 为节点号
 * 返回：无
 * 说明：之后需 mwSHS_clear_errors 清错误才能恢复。
 */
static inline void mwSHS_estop(uint8_t b, uint8_t n)
{ mwSHS_send(b, n, MWSHS_CMD_ESTOP, NULL, 0); }
/* mwSHS_set_state：发送 0x007 设置轴状态
 * 参数：b、n 为总线号、节点号；state 取 mwSHS_state_t
 * 返回：无
 * 说明：切换结果由心跳 B4 确认。
 */
static inline void mwSHS_set_state(uint8_t b, uint8_t n, uint32_t state)
{ mwSHS_send(b, n, MWSHS_CMD_STATE, &state, 4); }
/* mwSHS_set_mode：发送 0x00B 设置控制模式及输入模式
 * 参数：b、n 为总线号、节点号；ctrl 取 mwSHS_control_t；input 取 mwSHS_input_t
 * 返回：无
 * 说明：本工程力矩模式用 1+1，参考工程位置模式用 3+5。
 */
static inline void mwSHS_set_mode(uint8_t b, uint8_t n, uint32_t ctrl, uint32_t input)
{ uint32_t p[2] = {ctrl, input}; mwSHS_send(b, n, MWSHS_CMD_MODE, p, 8); }
/* mwSHS_set_limits：发送 0x00F 设置限速和限流
 * 参数：b、n 为总线号、节点号；vel 为转子侧限速 turns/s；cur 为限流 A
 * 返回：无
 * 说明：载荷依次为限速、限流。
 */
static inline void mwSHS_set_limits(uint8_t b, uint8_t n, float vel, float cur)
{ float p[2] = {vel, cur}; mwSHS_send(b, n, MWSHS_CMD_LIMITS, p, 8); }
/* mwSHS_pos：发送 0x00C 位置及前馈指令
 * 参数：b、n 为总线号、节点号；pos 为转子侧 turns；vel_ff 单位 0.001 turns/s；
 *       tq_ff 单位 0.001 Nm
 * 返回：无
 * 说明：两个前馈均为 int16 定点数，tq_ff 填 200 即 0.2 Nm。
 */
static inline void mwSHS_pos(uint8_t b, uint8_t n, float pos, int16_t vel_ff, int16_t tq_ff)
{ uint8_t p[8]; memcpy(p, &pos, 4); memcpy(p+4, &vel_ff, 2); memcpy(p+6, &tq_ff, 2); mwSHS_send(b, n, MWSHS_CMD_POS, p, 8); }
/* mwSHS_vel：发送 0x00D 速度及力矩前馈指令
 * 参数：b、n 为总线号、节点号；vel 为转子侧 turns/s；tq_ff 为力矩前馈 Nm
 * 返回：无
 * 说明：速度及前馈均为 32 位浮点数。
 */
static inline void mwSHS_vel(uint8_t b, uint8_t n, float vel, float tq_ff)
{ float p[2] = {vel, tq_ff}; mwSHS_send(b, n, MWSHS_CMD_VEL, p, 8); }
/* mwSHS_torque：发送 0x00E 力矩指令
 * 参数：b、n 为总线号、节点号；tq 为转子侧力矩 Nm
 * 返回：无
 * 说明：关节力矩需先除以减速比，见文件头。
 */
static inline void mwSHS_torque(uint8_t b, uint8_t n, float tq)
{ mwSHS_send(b, n, MWSHS_CMD_TORQUE, &tq, 4); }
/* mwSHS_set_traj_vel：发送 0x011 设置梯形轨迹限速
 * 参数：b、n 为总线号、节点号；v 为转子侧限速 turns/s
 * 返回：无
 * 说明：用于梯形轨迹输入模式。
 */
static inline void mwSHS_set_traj_vel(uint8_t b, uint8_t n, float v)
{ mwSHS_send(b, n, MWSHS_CMD_TRAJ_VEL, &v, 4); }
/* mwSHS_set_traj_accel：发送 0x012 设置梯形轨迹加减速限值
 * 参数：b、n 为总线号、节点号；acc、dec 为加速、减速限值 turns/s²
 * 返回：无
 * 说明：用于梯形轨迹输入模式。
 */
static inline void mwSHS_set_traj_accel(uint8_t b, uint8_t n, float acc, float dec)
{ float p[2] = {acc, dec}; mwSHS_send(b, n, MWSHS_CMD_TRAJ_ACCEL, p, 8); }
/* mwSHS_set_traj_inertia：发送 0x013 设置梯形轨迹惯量
 * 参数：b、n 为总线号、节点号；j 为惯量 Nm/(turns/s²)
 * 返回：无
 * 说明：用于梯形轨迹输入模式。
 */
static inline void mwSHS_set_traj_inertia(uint8_t b, uint8_t n, float j)
{ mwSHS_send(b, n, MWSHS_CMD_TRAJ_INERTIA, &j, 4); }
/* mwSHS_set_pos_gain：发送 0x01A 设置位置环增益
 * 参数：b、n 为总线号、节点号；g 为位置环增益 (turns/s)/turn
 * 返回：无
 * 说明：位置误差到速度指令的比例增益。
 */
static inline void mwSHS_set_pos_gain(uint8_t b, uint8_t n, float g)
{ mwSHS_send(b, n, MWSHS_CMD_POS_GAIN, &g, 4); }
/* mwSHS_set_vel_gain：发送 0x01B 设置速度环增益
 * 参数：b、n 为总线号、节点号；g、ig 为速度环比例增益 Nm/(turns/s)、积分增益 Nm/turn
 * 返回：无
 * 说明：载荷依次为比例、积分增益。
 */
static inline void mwSHS_set_vel_gain(uint8_t b, uint8_t n, float g, float ig)
{ float p[2] = {g, ig}; mwSHS_send(b, n, MWSHS_CMD_VEL_GAIN, p, 8); }
/* mwSHS_clear_errors：发送 0x018 清除错误
 * 参数：b 为总线号；n 为节点号
 * 返回：无
 * 说明：急停后恢复闭环前需先清错误。
 */
static inline void mwSHS_clear_errors(uint8_t b, uint8_t n)
{ mwSHS_send(b, n, MWSHS_CMD_CLEAR_ERRORS, NULL, 0); }
/* mwSHS_reboot：发送 0x016 重启电调
 * 参数：b 为总线号；n 为节点号
 * 返回：无
 * 说明：发送 8 字节全 0 载荷。
 */
static inline void mwSHS_reboot(uint8_t b, uint8_t n)
{ mwSHS_send(b, n, MWSHS_CMD_REBOOT, NULL, 0); }
/* mwSHS_save_config：发送 0x01F 保存配置并重启
 * 参数：b 为总线号；n 为节点号
 * 返回：无
 * 说明：保存后会重启电机，不用于常规上电流程。
 */
static inline void mwSHS_save_config(uint8_t b, uint8_t n)
{ mwSHS_send(b, n, MWSHS_CMD_SAVE, NULL, 0); }
/* mwSHS_request：发送 8 字节全 0 查询帧
 * 参数：b、n 为总线号、节点号；cmd 为查询命令码
 * 返回：无
 * 说明：电调用同一 ID 异步回复，回复到达前 mwSHS_get 仍是旧值。
 *       ERROR（0x003）默认查询电机错误，查其他类型用 mwSHS_send 指定 B0。
 */
static inline void mwSHS_request(uint8_t b, uint8_t n, uint8_t cmd)
{ mwSHS_send(b, n, cmd, NULL, 0); }
#endif
