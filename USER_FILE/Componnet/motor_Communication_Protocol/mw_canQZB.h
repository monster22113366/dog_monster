/* Motorevo 单独控制；单位 rad、rad/s、Nm，量程对应参数 24..35。 */
#ifndef MW_CANQZB_H
#define MW_CANQZB_H
#include <stdint.h>
#include <stddef.h>

/* 独立协议帧；适配层逐字段转换，不依赖平台布局。 */
typedef struct {
    uint32_t id;
    uint8_t len;
    uint8_t data[64];
    uint8_t extended, fd, brs;
} mwQZB_frame_t;
typedef int (*mwQZB_tx_fn)(uint8_t bus, const mwQZB_frame_t *frame);
typedef uint32_t (*mwQZB_lock_fn)(void);
typedef void (*mwQZB_unlock_fn)(uint32_t key);
typedef uint32_t (*mwQZB_millis_fn)(void);
#define MWQZB_BUS_NUM 3u
typedef struct {
    mwQZB_tx_fn tx;
    mwQZB_millis_fn millis;
    mwQZB_lock_fn lock;
    mwQZB_unlock_fn unlock;
} mwQZB_ops_t;


#ifndef MWQZB_USE_FD
#define MWQZB_USE_FD 1
#endif
/* 实机:5M 数据段发送在本硬件上导致 bus-off,整帧 1M 稳定;电机反馈仍可为 BRS 帧,RX 不受影响。 */
#ifndef MWQZB_USE_BRS
#define MWQZB_USE_BRS 0
#endif
#define MWQZB_MAX_ID 8u
#define MWQZB_STATE_ID_BASE 0x100u
#define MWQZB_CONTROL_ID_BASE 0x200u
#define MWQZB_PARAM_ID_BASE 0x600u
#define MWQZB_PARAM_CONTROL_MODE 11u
#ifndef MWQZB_FAULT_MASK
#define MWQZB_FAULT_MASK 0x3D1Eu
#endif
#ifndef MWQZB_POS_MIN
#define MWQZB_POS_MIN (-12.5f)
#endif
#ifndef MWQZB_POS_MAX
#define MWQZB_POS_MAX 12.5f
#endif
#ifndef MWQZB_VEL_MIN
#define MWQZB_VEL_MIN (-65.0f)
#endif
#ifndef MWQZB_VEL_MAX
#define MWQZB_VEL_MAX 65.0f
#endif
#ifndef MWQZB_KP_MIN
#define MWQZB_KP_MIN 0.0f
#endif
#ifndef MWQZB_KP_MAX
#define MWQZB_KP_MAX 500.0f
#endif
#ifndef MWQZB_KD_MIN
#define MWQZB_KD_MIN 0.0f
#endif
#ifndef MWQZB_KD_MAX
#define MWQZB_KD_MAX 5.0f
#endif
#ifndef MWQZB_KI_MIN
#define MWQZB_KI_MIN 0.0f
#endif
#ifndef MWQZB_KI_MAX
#define MWQZB_KI_MAX 0.05f
#endif
#ifndef MWQZB_TQ_MIN
#define MWQZB_TQ_MIN (-18.0f)
#endif
#ifndef MWQZB_TQ_MAX
#define MWQZB_TQ_MAX 18.0f
#endif

typedef enum { MWQZB_MODE_MIT = 2, MWQZB_MODE_VELOCITY = 3 } mwQZB_mode_t;
typedef enum {
    MWQZB_STATE_QUERY = 0xFA, MWQZB_STATE_CLEAR = 0xFB,
    MWQZB_STATE_ENABLE = 0xFC, MWQZB_STATE_DISABLE = 0xFD,
    MWQZB_STATE_ZERO = 0xFE /* 写 Flash，禁止周期发送。 */
} mwQZB_state_cmd_t;
typedef struct {
    float pos, vel, torque;
    int16_t temperature;
    uint16_t status, error;
    uint8_t enabled;
    uint32_t count, time; /* 最近反馈计数及 平台毫秒时钟 时间。 */
} mwQZB_feedback_t;
typedef struct { uint8_t index; uint32_t raw, count; } mwQZB_param_t;

/* 保存平台能力副本并清空全部缓存及登记状态；不初始化硬件。
 * ops/tx 缺失时发送返回 -1，millis 缺失时返回 0；
 * lock/unlock 任一缺失时整对停用。仅启动期调用。 */
void mwQZB_init(const mwQZB_ops_t *ops);
/* 清空指定总线缓存并登记其 ID 1..8；越界为空操作。 */
void mwQZB_bus_register(uint8_t bus);
/* 原样返回注入的发送结果：1 入队、0 FIFO 满、-1 错误。 */
int mwQZB_send(uint8_t bus, uint16_t ident_base, uint8_t id, const uint8_t data[8]);
/* 调用方保证输入有限；此处仅按编译期量程饱和、截断量化。 */
void mwQZB_pack_mit(uint8_t d[8], float pos, float vel, float kp, float kd, float tq);
void mwQZB_pack_vel(uint8_t d[8], float vel, float kp, float kd, float ki);
void mwQZB_pack_state(uint8_t d[8], mwQZB_state_cmd_t cmd);
void mwQZB_pack_param(uint8_t d[8], uint8_t index, uint32_t raw);
void mwQZB_recv(uint8_t bus, const mwQZB_frame_t *frame);
/* 临界区内一致拷贝，非法总线/ID/指针返回 0，否则返回 1。 */
int mwQZB_get_feedback(uint8_t bus, uint8_t id, mwQZB_feedback_t *out);
int mwQZB_get_param(uint8_t bus, uint8_t id, mwQZB_param_t *out);
#endif
