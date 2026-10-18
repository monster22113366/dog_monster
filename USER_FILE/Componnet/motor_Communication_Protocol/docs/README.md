# MW 电调 CAN 通信协议说明

本文面向第一次接触这款电机的读者。读完后应该能够：

- 看懂总线上每一帧的含义；
- 不依赖任何 SDK，自己写出发送和接收函数。

**事实来源**：本目录的 `mw_canSHS.h` / `mw_canSHS.c`（已验证的精简实现）、厂商 SDK `MWMotor.h` / `MWMotor.c`，以及参考工程 26Initial-RC-dog 的实测结论。代码里找不到依据的内容，统一标注 **【需向厂商确认】**。

---

## 目录

1. [概述](#1-概述)
2. [物理层](#2-物理层)
3. [CAN ID 结构](#3-can-id-结构)
4. [数据域编码](#4-数据域编码)
5. [完整命令表](#5-完整命令表)
6. [发送：模式、状态机与运动指令](#6-发送模式状态机与运动指令)
7. [接收解析](#7-接收解析)
8. [上电与停机流程](#8-上电与停机流程)
9. [帧示例（逐字节）](#9-帧示例逐字节)
10. [从零重写收发函数](#10-从零重写收发函数)
11. [与 mw_canSHS.h/.c 的对应关系](#11-与-mw_canshshc-的对应关系)
12. [参考链接](#12-参考链接)

---


## 1. 概述

本驱动只实现 CANSimple 子集，0x008 MIT 及 input mode 9 不支持。

| 项目 | 说明 |
|---|---|
| 电调 | 北京守护兽科技（CyberBeast）MW 系列伺服电调 |
| 协议来源 | ODrive CANSimple（0.5.x 固件那一代）加厂商扩展 |
| 厂商扩展 | 心跳布局采用 SDK 中 `FIRMWARE_VERSION_NUMBER >= 513` 的分支（SDK 头文件里定义的值为 605） |
| 适用范围 | 与 SDK `MWMotor.c` 行为一致的 MW 电调固件。其他固件版本的心跳布局不同（SDK 里有 `<=511` 和 `==512` 两个分支），需要先向厂商确认固件版本 |

### 与 ODrive 0.6.x 官方文档的差异（重要）

网上最容易找到的是 ODrive **0.6.x** 的 CAN 文档，但它和本电调**有几处不兼容**，不能照抄：

| 项目 | 本电调（按 SDK） | 说明 |
|---|---|---|
| 心跳 B5 | 错误标志位与轨迹完成位（见 [§7.1](#71-周期心跳0x001)） | 与 0.6.x 文档的定义不同 |
| `0x003` | 请求帧 B0 是错误类型，回复帧 B0~B3 是 u32 错误码 | 与 0.6.x 的 Get_Error 布局不同 |
| `0x006` | B0 是新的 node id | 与 0.6.x 的定义不同 |
| `0x004` / `0x005` | SDK 的做法：读用 0x004 发，写用 0x005 发，回复在 0x004 上 | 与 0.6.x 的 RxSdo/TxSdo 语义不同【需向厂商确认】 |

其他命令与 0.6.x 的差异没有逐条核对。**一律以本文和 SDK 为准。**

---

## 2. 物理层

| 项目 | 取值 |
|---|---|
| 协议 | 经典 CAN 2.0，**不是 CAN FD** |
| DLC | **固定为 8**。下行帧不足 8 字节时补 0；接收时 DLC≠8 的帧直接丢弃（`mwSHS_recv` 就是这样做的） |
| 波特率 | **以电调配置为准**。参考工程用的是 1 Mbps：FDCAN 时钟 160 MHz，prescaler=4，位时间 = 1（同步段）+ 31（TSEG1）+ 8（TSEG2）= 40 tq，即 160 MHz / 4 / 40 = 1 Mbps，采样点 (1+31)/40 = 80% |
| 帧类型 | **可能是标准帧，也可能是扩展帧**（参考工程实测两种都遇到过），但 ID 数值都 ≤ 0x7FF |
| 远程帧 RTR | 不使用。查询请求是 8 字节全 0 的**数据帧**（见 [§5.2](#52-查询请求的形式)） |

### 帧类型的处理办法（参考工程的做法）

1. 对方的帧类型未知时，每条指令**标准帧、扩展帧各发一次**（ID 数值相同）。
2. 收到该电机的心跳（0x001）或编码器回包（0x009）后，记下它用的帧类型（IDE 位），之后只发这一种。
3. 接收过滤器要**同时放行标准帧和扩展帧**。扩展帧取 `ExtId`，其数值 ≤ 0x7FF，按 11 位 ID 同样解析。

> 注意：`mw_canSHS.h` 顶部的使用示例只放行了 `CAN_ID_STD`。如果你的电机用扩展帧，要在应用层把 `ExtId`（≤0x7FF）也传给 `mwSHS_recv`。帧类型由你的发送回调决定，`mw_canSHS` 本身不关心。

---

## 3. CAN ID 结构

### 3.1 位图

```
      10   9   8   7   6   5   4   3   2   1   0     ← ID 位号（11 位）
    +---+---+---+---+---+---+---+---+---+---+---+
    |       node_id (6 bit)     |  cmd_id (5 bit)  |
    +---+---+---+---+---+---+---+---+---+---+---+
      node: 0 ~ 63                cmd: 0x00 ~ 0x1F
```

### 3.2 公式

```c
id      = (node_id << 5) | cmd_id;   // 发送
node_id = id >> 5;                   // 接收
cmd_id  = id & 0x1F;
```

- **上行和下行用同一个 ID**。例如给 node1 发 0x009 查询，电调也用 ID 0x029 回复。
- 最大 ID 为 (63 << 5) | 31 = 0x7FF，正好占满 11 位。
- 示例：node1 的位置指令 ID = (1<<5)|0x0C = **0x02C**；node2 的心跳 ID = (2<<5)|0x01 = **0x041**。

### 3.3 ID 速查表（node 1~4）

| cmd | 名称 | node1 | node2 | node3 | node4 |
|---|---|---|---|---|---|
| 0x001 | 心跳 | 0x021 | 0x041 | 0x061 | 0x081 |
| 0x002 | 急停 | 0x022 | 0x042 | 0x062 | 0x082 |
| 0x007 | 设轴状态 | 0x027 | 0x047 | 0x067 | 0x087 |
| 0x009 | 编码器估计 | 0x029 | 0x049 | 0x069 | 0x089 |
| 0x00B | 设控制/输入模式 | 0x02B | 0x04B | 0x06B | 0x08B |
| 0x00C | 位置指令 | 0x02C | 0x04C | 0x06C | 0x08C |
| 0x00D | 速度指令 | 0x02D | 0x04D | 0x06D | 0x08D |
| 0x00E | 力矩指令 | 0x02E | 0x04E | 0x06E | 0x08E |
| 0x018 | 清错误 | 0x038 | 0x058 | 0x078 | 0x098 |

（表中数值由临时程序按上面的公式打印后核对过。）

---

## 4. 数据域编码

本驱动支持的数据帧采用小端编码，float 为 IEEE-754 单精度。

### 4.1 普通帧（小端）

一帧通常是"两个 32 位量"或"一个 32 位量加补 0"：

```
 字节:   B0      B1      B2      B3      B4      B5      B6      B7
       +-------+-------+-------+-------+-------+-------+-------+-------+
       | a[7:0]|a[15:8]|a[23:16]|a[31:24]| b[7:0]|b[15:8]|b[23:16]|b[31:24]|
       +-------+-------+-------+-------+-------+-------+-------+-------+
       \_______ 字段 a (u32/i32/f32) ______/\_______ 字段 b (u32/i32/f32) ______/
```

例：float 1.5 = 0x3FC00000，小端依次存为 `00 00 C0 3F`。

0x00C 是例外，后 4 字节拆成两个 int16：

```
 字节:   B0   B1   B2   B3  |  B4     B5   |  B6     B7
       [ input_pos  f32  ]  | vel_ff i16  | tq_ff  i16
                             0.001 turns/s  0.001 Nm
```

在小端 MCU（STM32 等）上可以直接 `memcpy` 结构体字段。大端主机要手动拆字节。

---

## 5. 完整命令表

### 5.1 约定

- **方向**：
  - 下行：主控发，电调不回。
  - 请求-应答：主控发全 0 的请求，电调用同一 ID 回复。
  - 周期上报：电调主动发。
- 未特别说明时，字段均为小端；"补 0"表示该字节发 0x00、收时忽略。
- 普通帧的位置和速度是**电机转子侧（减速前）**的值，单位 turns、turns/s。

| cmd | 名称 | 方向 | B0~B3 | B4~B7 | 单位 / 备注 |
|---|---|---|---|---|---|
| 0x000 | — | — | — | — | SDK 未定义，不使用 |
| 0x001 | 心跳 Heartbeat | 周期上报 | u32 axis_error | B4 当前状态；B5 标志位；B6 未使用；B7 life 计数 | 周期由电调配置决定；逐位含义见 [§7.1](#71-周期心跳0x001) |
| 0x002 | 急停 Estop | 下行 | 全 0 | 全 0 | 急停后需清错误（0x018）才能恢复 |
| 0x003 | 查询错误 Get_Error | 请求-应答 | 请求：B0 = 错误类型（0 电机，1 编码器，2 无感，3 控制器），其余补 0<br>回复：u32 错误码 | 回复：未使用 | 错误码的含义【需向厂商确认】 |
| 0x004 | SDO（RxSdo） | 请求-应答 | B0 opcode（0 读，1 写）；B1~B2 u16 endpoint id；B3 保留 | int32 或 float 数据 | SDK 用它发"读"请求，回复也在这个 ID 上解析。详见 [§5.3](#53-sdo0x0040x005)【需向厂商确认】 |
| 0x005 | SDO（TxSdo） | 下行 | 布局同 0x004 | 同 0x004 | SDK 用它发"写"请求【需向厂商确认】 |
| 0x006 | 设节点号 Set_Axis_Node_ID | 下行 | B0 = 新 node id（0~63），其余补 0 | 补 0 | 是否需要 0x01F 保存、何时生效【需向厂商确认】 |
| 0x007 | 设轴状态 Set_Axis_State | 下行 | u32 目标状态（见 [§6.2](#62-轴状态0x007)） | 补 0 | 常用：1 IDLE，8 CLOSED_LOOP |
| 0x009 | 编码器估计 Get_Encoder_Estimates | 请求-应答 | 回复：f32 pos | 回复：f32 vel | turns、turns/s（转子侧）。是否也会周期上报【需向厂商确认】 |
| 0x00A | 编码器计数 Get_Encoder_Count | 请求-应答 | 回复：i32 shadow_count（多圈计数） | 回复：i32 count_in_cpr（单圈计数） | counts |
| 0x00B | 设模式 Set_Controller_Mode | 下行 | u32 控制模式（0 电压，1 力矩，2 速度，3 位置） | u32 输入模式（0 IDLE，1 直接，2 速度斜坡，3 位置滤波，5 梯形轨迹，6 力矩斜坡） | — |
| 0x00C | 位置指令 Set_Input_Pos | 下行 | f32 input_pos | B4~B5 i16 vel_ff；B6~B7 i16 tq_ff | pos: turns；vel_ff: 0.001 turns/s；tq_ff: 0.001 Nm |
| 0x00D | 速度指令 Set_Input_Vel | 下行 | f32 input_vel | f32 tq_ff | turns/s；Nm |
| 0x00E | 力矩指令 Set_Input_Torque | 下行 | f32 input_torque | 补 0 | Nm |
| 0x00F | 设限值 Set_Limits | 下行 | f32 vel_limit | f32 current_limit | turns/s；A |
| 0x010 | 齿槽转矩校准 Start_Anticogging | 下行 | 全 0 | 全 0 | 电机会转动，执行前注意安全 |
| 0x011 | 梯形轨迹限速 Set_Traj_Vel_Limit | 下行 | f32 vel_limit | 补 0 | turns/s |
| 0x012 | 梯形轨迹加减速 Set_Traj_Accel_Limits | 下行 | f32 accel_limit | f32 decel_limit | turns/s²（由 turns/s 推出）【需向厂商确认】 |
| 0x013 | 梯形轨迹惯量 Set_Traj_Inertia | 下行 | f32 inertia | 补 0 | 单位【需向厂商确认】 |
| 0x014 | 电流 Get_Iq | 请求-应答 | 回复：f32 iq_setpoint | 回复：f32 iq_measured | A |
| 0x015 | 无感估计 Get_Sensorless_Estimates | — | — | — | SDK 头文件里声明了，但 .c 里没有实现，`mw_canSHS` 也不支持【需向厂商确认】 |
| 0x016 | 重启 Reboot | 下行 | 全 0 | 全 0 | — |
| 0x017 | 母线电压电流 Get_Bus_Voltage_Current | 请求-应答 | 回复：f32 bus_voltage | 回复：f32 bus_current | V；A |
| 0x018 | 清错误 Clear_Errors | 下行 | 全 0 | 全 0 | — |
| 0x019 | 设编码器计数 Set_Linear_Count | 下行 | i32 linear_count | 补 0 | counts |
| 0x01A | 位置环增益 Set_Pos_Gain | 下行 | f32 pos_gain | 补 0 | 单位【需向厂商确认】 |
| 0x01B | 速度环增益 Set_Vel_Gains | 下行 | f32 vel_gain | f32 vel_integrator_gain | 单位【需向厂商确认】 |
| 0x01C | 力矩 Get_Torques | 请求-应答 | 回复：f32 torque_setpoint | 回复：f32 torque | Nm |
| 0x01D | 功率 Get_Powers | 请求-应答 | 回复：f32 electrical_power | 回复：f32 mechanical_power | 单位【需向厂商确认】（按 ODrive 惯例为 W） |
| 0x01E | 禁用 CAN Disable_CAN | 下行 | 全 0 | 全 0 | 禁用后如何恢复通信【需向厂商确认】，慎用 |
| 0x01F | 保存配置 Save_Configuration | 下行 | 全 0 | 全 0 | SDK 注释："保存电机设置并重启电机" |

### 5.2 查询请求的形式

- 参考 SDK 和参考工程发的都是 **8 字节全 0 的数据帧**，不用 RTR；电调用**同一个 ID** 回复。
- ODrive 官方文档提到可以用 RTR 查询，本电调是否支持【需向厂商确认】。
- 0x003 是例外，请求帧的 B0 要填错误类型。
- **回复是异步到达的**。SDK 的 `MWGetXxx()` 发出请求后立刻返回表里**上一次缓存的值**，不是本次请求的结果。

### 5.3 SDO（0x004/0x005）

```
 B0        B1   B2          B3     B4   B5   B6   B7
 opcode  | endpoint id  |  保留  |   data (int32 或 float，小端)  |
 0读/1写 | u16 小端     |        |                                |
```

- SDK 的行为：读请求用 0x004 发，写请求用 0x005 发，回复在 0x004 上解析（B0 opcode、B1~B2 id、B4~B7 data）。
- data 是 int32 还是 float，取决于 endpoint，回复里没有类型字段。SDK 的做法是在本地记住请求时的类型。
- endpoint id 表需要到厂商官网下载 JSON：https://www.cyberbeast.cn/filedownload/837298
- 以上 SDO 细节（写操作是否有回复、回复具体用哪个 cmd 等）**【需向厂商确认】**。`mw_canSHS` 不解析 SDO 回包，需要时用 `mwSHS_send` 自己组帧。

---

## 6. 发送：模式、状态机与运动指令

### 6.1 控制模式 × 输入模式

0x00B 同时设置两者：B0~B3 为控制模式，B4~B7 为输入模式。**控制模式决定闭哪几个环，输入模式决定指令怎么进入这几个环。**

| 控制模式 (B0) | 输入模式 (B4) | 下发指令 | 行为 |
|---|---|---|---|
| 3 位置 | 5 梯形轨迹 | 0x00C | 电调按 0x011~0x013 规划梯形轨迹，内部闭位置、速度、电流三环。心跳 B5.bit7 表示轨迹完成。**参考工程实际使用** |
| 3 位置 | 1 直接 | 0x00C | 目标位置直接作为设定值，可带 vel_ff / tq_ff 前馈 |
| 3 位置 | 3 位置滤波 | 0x00C | 对目标位置做滤波，滤波参数需通过 SDO 设置【需向厂商确认】 |
| 2 速度 | 1 直接 | 0x00D | 目标速度直接生效，可带 tq_ff |
| 2 速度 | 2 速度斜坡 | 0x00D | 按斜率逼近目标速度，斜率参数需通过 SDO 设置【需向厂商确认】 |
| 1 力矩 | 1 直接 | 0x00E | 目标力矩直接生效 |
| 1 力矩 | 6 力矩斜坡 | 0x00E | 按斜率逼近目标力矩，斜率参数需通过 SDO 设置【需向厂商确认】 |
| 0 电压 | — | — | 用法【需向厂商确认】 |
| 任意 | 0 IDLE | — | 不接受输入 |

- 除"参考工程实际使用"的组合外，其余组合的语义按 ODrive 0.5 推断，是否全部可用【需向厂商确认】。

### 6.2 轴状态（0x007）

状态值（SDK 中的 `MWSHS_MOTER_STATE`，`mw_canSHS.h` 中的 `mwSHS_state_t`）：

| 值 | 名称 | 含义（SDK 注释） |
|---|---|---|
| 0 | UNDEFINED | 未定义 |
| 1 | **IDLE** | 空闲，PWM 关闭 |
| 2 | STARTUP_SEQUENCE | 启动序列 |
| 3 | FULL_CALIBRATION_SEQUENCE | 电机与编码器完整校准 |
| 4 | MOTOR_CALIBRATION | 测相电阻、相电感 |
| 6 | ENCODER_INDEX_SEARCH | 寻找编码器 index |
| 7 | ENCODER_OFFSET_CALIBRATION | 编码器偏移校准 |
| 8 | **CLOSED_LOOP_CONTROL** | 闭环控制 |
| 9 | LOCKIN_SPIN | 锁定自转（需已校准） |
| 10 | ENCODER_DIR_FIND | 编码器方向搜索（需已校准） |
| 11 | HOMING | 回零 |
| 12 | ENCODER_HALL_POLARITY_CALIBRATION | 霍尔极性校准 |
| 13 | ENCODER_HALL_PHASE_CALIBRATION | 霍尔相位校准 |
| 14 | ANTICOGGING_CALIBRATION | 齿槽转矩校准 |

（值 5 在 SDK 中没有定义。）

状态转换图（实线是本文确认过的用法；标 `?` 的边按 ODrive 0.5 的行为推断，【需向厂商确认】）：

```
                  上电
                   │
                   ▼
            ┌─────────────┐   0x007(8)    ┌────────────────────┐
            │   1 IDLE    │──────────────▶│ 8 CLOSED_LOOP      │
            │ 不响应运动  │◀──────────────│ 响应 0x00C/D/E     │
            │ 指令        │   0x007(1)    └────────────────────┘
            └─────────────┘                     │    │
              ▲    │  ▲                         │    │ 0x002 急停
              │    │  │ 出错 ?                  │    │ 或发生错误 ?
              │    │  └─────────────────────────┘    │
              │    │ 0x007(3/4/7/…)                   ▼
              │    ▼                           ┌────────────────┐
     完成 ?   │ ┌──────────────────┐           │ 带错误的 IDLE  │
     ─────────┘ │ 校准类状态       │           │ 心跳 axis_error│
                │ 3/4/6/7/10…14    │           │ ≠0 或 B5 有标志│
                └──────────────────┘           └────────────────┘
                                                        │ 0x018 清错误
                                                        ▼
                                                  回到 1 IDLE，
                                                  再 0x007(8) 恢复
```

要点：

- **电调在 IDLE 下不响应运动指令**。先切到 CLOSED_LOOP，并用心跳 B4==8 确认之后，再发运动指令。
- 急停（0x002）之后必须先 0x018 清错误，才能重新进入闭环。
- 状态切换是否成功，只能从心跳 B4 判断；0x007 本身没有回复。

### 6.3 各运动指令的用法

前提：已用 0x00B 设好对应模式，心跳 B4==8。

**位置（0x00C），配 3+5 梯形轨迹或 3+1 直接**

```c
mwSHS_pos(bus, node, 1.5f /*turns*/, 0 /*vel_ff ×0.001 turns/s*/, 0 /*tq_ff ×0.001 Nm*/);
```

- 使用梯形轨迹时，事先用 0x011（限速）、0x012（加减速）、0x013（惯量）设置轨迹参数，并用 0x00F 设置限速、限流。
- 前馈是 int16 定点数：想加 0.2 Nm 的力矩前馈，填 200。

**速度（0x00D），配 2+1 或 2+2**

```c
mwSHS_vel(bus, node, 2.0f /*turns/s*/, 0.0f /*tq_ff Nm*/);
```

**力矩（0x00E），配 1+1 或 1+6**

```c
mwSHS_torque(bus, node, 1.0f /*Nm*/);
```

所有运动指令都要**周期性发送**（例如在固定周期的控制任务里）。电调端是否有指令超时保护【需向厂商确认】。

---

## 7. 接收解析

接收到的帧分两类。统一的前置步骤：

1. DLC 必须为 8，否则丢弃。
2. 取 ID（标准帧取 StdId，扩展帧取 ExtId，数值都 ≤ 0x7FF）。
3. `node = id >> 5`，`cmd = id & 0x1F`，按 node 找到该电机的状态表。
4. 按 cmd 分发，解析完成后更新"最后接收时间"或接收计数。

### 7.1 周期心跳（0x001）

由电调主动、周期性上报，周期由电调配置决定。

| 字节 | 类型 | 含义 |
|---|---|---|
| B0~B3 | u32 小端 | axis_error（轴错误码，0 表示无错误） |
| B4 | u8 | 当前轴状态（见 [§6.2](#62-轴状态0x007)） |
| B5 | 位域 | bit0 电机错误；bit1 编码器错误；bit2 控制器错误；bit3 系统错误；bit4~6 未定义；**bit7 梯形轨迹完成** |
| B6 | — | 未使用 |
| B7 | u8 | life 计数 |

解析步骤：

```c
memcpy(&axis_error, &d[0], 4);
state          = d[4];
motor_err      = d[5]       & 1;
encoder_err    = (d[5] >> 1) & 1;
controller_err = (d[5] >> 2) & 1;
system_err     = (d[5] >> 3) & 1;
traj_done      = d[5] >> 7;
life           = d[7];
```

- life 的递增规律【需向厂商确认】。可以用来判断心跳是否是新的一帧。
- 各错误位只说明"有没有错"。要知道具体错误码，需用 0x003 按类型查询。

### 7.2 请求-应答型回包

对应 0x003、0x004、0x009、0x00A、0x014、0x017、0x01C、0x01D。都是小端，按 [§5](#5-完整命令表) 的表拷贝即可：

```c
case 0x009: memcpy(&pos, d, 4); memcpy(&vel, d + 4, 4); break;   // f32, f32
case 0x00A: memcpy(&shadow, d, 4); memcpy(&cpr, d + 4, 4); break; // i32, i32
case 0x003: memcpy(&err, d, 4); break;                            // u32
```

注意：

- 回包里**不带请求参数**。比如 0x003 回复不会告诉你查的是哪一类错误，调用方要自己记住上一次查询的类型。
- 回复是异步的。需要最新值时，先发请求，等收到回包（例如接收计数增加）后再读。

### 7.3 接收分发伪代码

```c
void on_can_rx(uint8_t bus, bool is_ext, uint32_t raw_id, const uint8_t d[8], uint8_t dlc)
{
    if (dlc != 8 || raw_id > 0x7FF) return;          // 非本协议帧
    uint16_t id = (uint16_t)raw_id;
    uint8_t node = id >> 5, cmd = id & 0x1F;
    motor_t *m = lookup(bus, node);
    if (!m) return;

    switch (cmd) {
    case 0x001: parse_heartbeat(m, d); m->frame_ext = is_ext; m->known = true; break;
    case 0x009: parse_f32_pair(&m->pos, &m->vel, d); m->frame_ext = is_ext; m->known = true; break;
    case 0x003: memcpy(&m->error, d, 8); break;
    case 0x00A: case 0x014: case 0x017: case 0x01C: case 0x01D:
                parse_pair(m, cmd, d); break;         // 小端 2×32 位
    default:    return;                               // 下行专用 cmd，忽略
    }
    m->last_rx_ms = now_ms();                         // 在线检测用
}
```

> 在中断里写、在任务里读时存在并发问题，见 [§10.3](#103-重写时的坑) 第 4 条。

### 7.4 在线检测：心跳超时

- 以心跳为准：`now − last_heartbeat_ms > 超时阈值` 即判定离线。阈值建议取心跳周期的 3~5 倍。心跳周期由电调配置决定，不确定时先实测。
- `mw_canSHS` 没有时间戳，只提供 `rx_count`（每成功解析一帧加 1，所有 cmd 共用）。应用层可以周期性检查 `rx_count` 是否变化：连续 N 个周期不变就判定离线。
- 离线后建议停止发送运动指令。重新上线后按 [§8](#8-上电与停机流程) 重新走一遍初始化。

---

## 8. 上电与停机流程

```
   主控 (Host)                                         电调 (node N)
      │                                                      │
      │ ◀──────────────── 0x001 心跳（周期） ─────────────── │  ① 等待心跳：确认在线，
      │                                                      │     记住帧类型（标准/扩展）
      │ ── 0x018 清错误 ───────────────────────────────────▶ │  ②
      │ ── 0x00B 控制模式 + 输入模式 ───────────────────────▶ │  ③ 例：3+5
      │ ── 0x00F 限速 + 限流 ───────────────────────────────▶ │  ④
      │ ── 0x011 / 0x012 / 0x013（仅梯形轨迹）──────────────▶ │
      │ ── 0x007 状态 = 8 CLOSED_LOOP ──────────────────────▶ │  ⑤
      │ ◀──────────────── 0x001 心跳 B4 == 8 ? ───────────── │  ⑥ 确认已进入闭环；
      │                                                      │     超时未到 8 则查错/重试
      │ ── 0x00C / 0x00D / 0x00E（周期）────────────▶ │  ⑦ 周期性发送控制指令
      │ ◀──────────────── 0x001 心跳（周期）──────────────── │
      │                        …                             │
      │ ── 0x007 状态 = 1 IDLE ─────────────────────────────▶ │  正常停机
      │                                                      │
      │ ── 0x002 急停 ──────────────────────────────────────▶ │  紧急情况
      │     （恢复：0x018 清错误 → 再走 ⑤⑥）                  │
```

要点：

- 第 ③、④ 步最好在 IDLE 下完成，之后再切闭环。在闭环中改模式是否安全【需向厂商确认】。
- 第 ⑥ 步必须等心跳确认。0x007 没有回复，发完不代表状态已经切换。
- 0x01F 保存配置会重启电机，不要放进常规上电流程。

---

## 9. 帧示例（逐字节）

以下所有字节都由本目录的 `mw_canSHS.c` 实际编译运行得到（gcc，x86 小端，`sizeof(float)=4`），不是手算的。格式：`ID  [DLC]  B0 … B7`。

### 9.1 node1 位置 1.5 turns（`mwSHS_pos(0,1,1.5f,0,0)`）

```
0x02C  [8]  00 00 C0 3F 00 00 00 00
```

| 字节 | 值 | 含义 |
|---|---|---|
| B0~B3 | `00 00 C0 3F` | f32 1.5（0x3FC00000，小端） |
| B4~B5 | `00 00` | vel_ff = 0 |
| B6~B7 | `00 00` | tq_ff = 0 |

ID：(1<<5)|0x0C = 0x02C。

### 9.2 node1 速度 2.0 turns/s，前馈 0（`mwSHS_vel(0,1,2.0f,0.0f)`）

```
0x02D  [8]  00 00 00 40 00 00 00 00
```

B0~B3 = f32 2.0（0x40000000）；B4~B7 = f32 0.0。

### 9.3 node1 设状态 CLOSED_LOOP（`mwSHS_set_state(0,1,8)`）

```
0x027  [8]  08 00 00 00 00 00 00 00
```

B0~B3 = u32 8；其余补 0。（设 IDLE：`0x027  [8]  01 00 00 00 00 00 00 00`）

### 9.4 node1 设模式：位置 + 梯形轨迹（`mwSHS_set_mode(0,1,3,5)`）

```
0x02B  [8]  03 00 00 00 05 00 00 00
```


B0~B3 = u32 3（位置控制）；B4~B7 = u32 5（梯形轨迹）。

### 9.5 node2 心跳回包：无错误，state=8，轨迹完成，life=42

```
0x041  [8]  00 00 00 00 08 80 00 2A
```

ID 解析：0x041 >> 5 = 2（node2）；0x041 & 0x1F = 0x01（心跳）。

| 字节 | 值 | 解析 |
|---|---|---|
| B0~B3 | `00 00 00 00` | axis_error = 0，无错误 |
| B4 | 08 | state = 8 CLOSED_LOOP |
| B5 | 80 | 0b1000_0000：bit0~3 = 0（电机/编码器/控制器/系统均无错误）；bit7 = 1，轨迹完成 |
| B6 | 00 | 未使用 |
| B7 | 2A | life = 42 |

把这帧喂给 `mwSHS_recv` 后，`mwSHS_get(0,2)` 得到：`axis_error=0 state=8 motor_err=0 encoder_err=0 controller_err=0 system_err=0 traj_done=1 life=42 rx_count=1`。

### 9.6 node1 编码器回包：pos=1.5，vel=−0.5

```
0x029  [8]  00 00 C0 3F 00 00 00 BF
```

| 字节 | 值 | 解析 |
|---|---|---|
| B0~B3 | `00 00 C0 3F` | f32 0x3FC00000 = 1.5 turns |
| B4~B7 | `00 00 00 BF` | f32 0xBF000000 = −0.5 turns/s |

喂给 `mwSHS_recv` 后，`mwSHS_get(0,1)` 得到 `pos=1.5 vel=-0.5`。对应的查询请求是 `0x029  [8]  00 00 00 00 00 00 00 00`，与回包同一个 ID。

### 9.7 补充示例

| 用途 | 调用 | 帧 |
|---|---|---|
| 清错误 | `mwSHS_clear_errors(0,1)` | `0x038  [8]  00 00 00 00 00 00 00 00` |
| 急停 | `mwSHS_estop(0,1)` | `0x022  [8]  00 00 00 00 00 00 00 00` |
| 限速 10 turns/s、限流 20 A | `mwSHS_set_limits(0,1,10,20)` | `0x02F  [8]  00 00 20 41 00 00 A0 41` |
| 力矩 1.0 Nm | `mwSHS_torque(0,1,1.0f)` | `0x02E  [8]  00 00 80 3F 00 00 00 00` |

---

## 10. 从零重写收发函数

### 10.1 最小骨架（约 50 行 C）

风格仿照 C610 那种"一发一收"：发送端按 cmd 组 8 字节，接收端在一个函数里按 cmd 分发。假设单总线、MCU 为小端、float 为 32 位 IEEE-754。

```c
/* mwSHS_min.c —— MW 电调最小收发骨架（单总线，小端 MCU） */
#include <stdint.h>
#include <string.h>

/* 平台实现：DLC=8；帧类型（标准/扩展）见 §2 */
extern void can_send(uint16_t id, const uint8_t d[8]);

typedef struct {
    uint32_t axis_error; uint8_t state, flags, life;  /* 0x001 心跳，flags = B5 原值 */
    float pos, vel;                                   /* 0x009，turns / turns/s */
    uint32_t rx_cnt;                                  /* 每解析一帧 +1，用于在线检测 */
} mwSHS_min_t;
static mwSHS_min_t mt[64];

const mwSHS_min_t *mwSHS_min_get(uint8_t node) { return &mt[node & 0x3F]; }

static void tx(uint8_t node, uint8_t cmd, const uint8_t d[8])
{ can_send((uint16_t)((node & 0x3F) << 5 | (cmd & 0x1F)), d); }

/* 两个 32 位字段：0x007(state,0) 0x00B(ctrl,input) 0x00D 0x00E 0x00F…；a/b 可传 &u32 或 &f32 */
void mwSHS_min_2x32(uint8_t node, uint8_t cmd, const void *a, const void *b)
{ uint8_t d[8] = {0}; if (a) memcpy(d, a, 4); if (b) memcpy(d + 4, b, 4); tx(node, cmd, d); }

void mwSHS_min_pos(uint8_t node, float pos, int16_t vff, int16_t tff)   /* 0x00C */
{ uint8_t d[8]; memcpy(d, &pos, 4); memcpy(d + 4, &vff, 2); memcpy(d + 6, &tff, 2); tx(node, 0x0C, d); }

void mwSHS_min_rx(uint16_t id, const uint8_t d[8], uint8_t dlc)   /* 在 CAN 接收回调里调用 */
{
    mwSHS_min_t *m;
    if (dlc != 8 || id > 0x7FF) return;
    m = &mt[id >> 5];
    switch (id & 0x1F) {
    case 0x01: memcpy(&m->axis_error, d, 4); m->state = d[4]; m->flags = d[5]; m->life = d[7]; break;
    case 0x09: memcpy(&m->pos, d, 4); memcpy(&m->vel, d + 4, 4); break;
    default: return;
    }
    m->rx_cnt++;
}
```

用法示例：

```c
uint32_t st = 8, ctrl = 3, in = 5; float vl = 10, il = 20;
mwSHS_min_2x32(1, 0x18, NULL, NULL);        // 清错误
mwSHS_min_2x32(1, 0x0B, &ctrl, &in);        // 位置 + 梯形
mwSHS_min_2x32(1, 0x0F, &vl, &il);          // 限速 10 turns/s，限流 20 A
mwSHS_min_2x32(1, 0x07, &st, NULL);         // CLOSED_LOOP
/* 等 mwSHS_min_get(1)->state == 8 */
mwSHS_min_pos(1, 1.5f, 0, 0);               // 周期性发送
```

这段骨架已与 `mw_canSHS.c` 对拍：[§9](#9-帧示例逐字节) 中的所有下行帧逐字节一致，心跳、编码器回包的解析结果也一致。其余请求-应答型回包（0x003/0x00A/0x014/0x017/0x01C/0x01D）照 `case 0x09` 的写法加上即可。

### 10.2 自检清单

- [ ] ID = `(node << 5) | cmd`，用 `uint16_t` 或更宽的类型存，**不能用 `uint8_t`**。
- [ ] 每帧 DLC=8，不足的部分补 0。
- [ ] 普通帧用小端，float 为 IEEE-754 单精度（`sizeof(float)==4`）。
- [ ] 0x00C 的后 4 字节是两个 **int16**（0.001 单位），不是 float。
- [ ] 查询请求发 8 字节全 0 的数据帧，不用 RTR。
- [ ] 接收端同时接受标准帧和扩展帧（ID ≤ 0x7FF），并记住对方的帧类型。
- [ ] 心跳按 ≥513 固件布局解析：B5 是位域，B7 是 life。
- [ ] 进闭环前先清错误、设模式、设限值；发 0x007(8) 后用心跳 B4==8 确认。
- [ ] 用心跳超时判断在线状态。
- [ ] 中断与任务之间共享的状态已做保护。
- [ ] 用 [§9](#9-帧示例逐字节) 的示例对拍自己的实现。

### 10.3 重写时的坑

1. **原 SDK 发送回调的 canId 类型是 `uint8_t`**（`typedef void (*MotorSender)(uint8_t busId, uint8_t canId, …)`）。node≥8 时 `(node<<5)|cmd` ≥ 0x100，ID 被截断，帧会发给错误的电机或发成错误的命令。重写时用 `uint16_t`（`mw_canSHS` 的 `mwSHS_tx_fn` 已经改成 `uint16_t`）。
2. **帧类型可能是标准帧，也可能是扩展帧**。只发、只收其中一种，就可能"电机不理你"或"收不到回包"。做法见 [§2](#2-物理层)。
3. **急停后需要清错误才能恢复**：0x002 之后直接发 0x007(8) 不会进入闭环，必须先 0x018。
4. **状态表在中断里写、在任务里读，存在并发问题**：一个 float 或多字段结构可能被读到一半新一半旧。可以在读取时关中断拷贝快照，或者用双缓冲、序列号。`mw_canSHS.h` 也注明了"由应用保护状态快照"。
5. **电调在 IDLE 下不响应运动指令**：没有报错，只是不动。先确认心跳 B4==8。
6. （补充）**Get 类函数返回的是缓存值**：SDK 的 `MWGetXxx()` 发出请求就返回，拿到的是上一次的回包。
7. （补充）**`mw_canSHS` 默认 `MWSHS_NODE_NUM=16`**：node≥16 的帧会被忽略，需要时编译时定义 `-DMW_NODE_NUM=64`。
8. （补充，推断）**上下行同 ID**：如果总线上有第二个主机，或者开了回环，主控自己发的 0x009 请求会被当作回包解析（例如全 0 的 0x009 请求会被解析成 pos=0、vel=0）。单主机、关闭回环时不受影响。

---

## 11. 与 mw_canSHS.h/.c 的对应关系

### 11.1 API 速查表

| cmd | mw_canSHS API | 原 SDK 函数 | 说明 |
|---|---|---|---|
| — | `mwSHS_init(tx)` | `MWRegisterMotor` | 注册发送回调并清空状态表 |
| — | `mwSHS_get(bus,node)` | `motorData` 指针 | 返回状态快照指针；越界时返回 NULL |
| 任意 | `mwSHS_send(bus,node,cmd,payload,len)` | — | 通用组帧：负载不足 8 字节补 0 |
| 接收 | `mwSHS_recv(bus,id,data,len)` | `MWReceiver` | 解析 0x001/003/009/00A/014/017/01C/01D；len≠8 丢弃 |
| 0x002 | `mwSHS_estop` | `MWEstop` | |
| 0x003 | `mwSHS_request(b,n,MWSHS_CMD_ERROR)`；其他类型用 `mwSHS_send` 填 B0 | `MWGetMotorError` | 回包存在 `error` 字段 |
| 0x004/0x005 | `mwSHS_send` 自己组帧 | `MWRxTxSdo` | `mw_canSHS` 不解析 SDO 回包 |
| 0x006 | `mwSHS_send(b,n,MWSHS_CMD_NODE_ID,&new_id,1)` | `MWSetAxisNodeID` | |
| 0x007 | `mwSHS_set_state` | `MWSetAxisState` | |
| 0x009 | `mwSHS_request(b,n,MWSHS_CMD_ENCODER)` | `MWGetEncoderEstimates` | `pos`、`vel` |
| 0x00A | `mwSHS_request(…,MWSHS_CMD_COUNT)` | `MWGetEncoderCount` | `shadow_count`、`count_in_cpr` |
| 0x00B | `mwSHS_set_mode` | `MWSetControllerMode` | |
| 0x00C | `mwSHS_pos` | `MWPosControl` | |
| 0x00D | `mwSHS_vel` | `MWVelControl` | |
| 0x00E | `mwSHS_torque` | `MWTorqueControl` | |
| 0x00F | `mwSHS_set_limits` | `MWSetLimits` | |
| 0x010 | `mwSHS_send(…,MWSHS_CMD_ANTICOGGING,NULL,0)` | `MWStartAnticogging` | |
| 0x013 | `mwSHS_set_traj_inertia` | `MWSetTrajInertia` | |
| 0x014 | `mwSHS_request(…,MWSHS_CMD_IQ)` | `MWGetIq` | `iq_set`、`iq_meas` |
| 0x016 | `mwSHS_reboot` | `MWReboot` | |
| 0x017 | `mwSHS_request(…,MWSHS_CMD_BUS)` | `MWGetBusVoltageCurrent` | `bus_v`、`bus_i` |
| 0x018 | `mwSHS_clear_errors` | `MWClearErrors` | |
| 0x019 | `mwSHS_send(…,MWSHS_CMD_MOVE_INC,&disp,4)` | `MWSetLinearCount` | `float disp`（转子侧 turns） |
| 0x01A | `mwSHS_set_pos_gain` | `MWSetPosGain` | |
| 0x01B | `mwSHS_set_vel_gain` | `MWSetVelGain` | |
| 0x01C | `mwSHS_request(…,MWSHS_CMD_TORQUES)` | `MWGetTorques` | `torque_set`、`torque` |
| 0x01D | `mwSHS_request(…,MWSHS_CMD_POWERS)` | `MWGetPowers` | `elec_power`、`mech_power` |
| 0x01E | `mwSHS_send(…,MWSHS_CMD_DISABLE_CAN,NULL,0)` | `MWDisableCAN` | |
| 0x01F | `mwSHS_save_config` | `MWSaveConfigeration` | |

状态表字段（`mwSHS_motor_t`）：`axis_error, state, motor_err, encoder_err, controller_err, system_err, traj_done, life, error, pos, vel, shadow_count, count_in_cpr, iq_set, iq_meas, bus_v, bus_i, torque_set, torque, elec_power, mech_power, rx_count`。

### 11.2 最小使用示例

```c
#include "mw_canSHS.h"

static void tx(uint8_t bus, uint16_t id, const uint8_t data[8])
{
    app_can_send(bus, id, data);   /* 应用负责：DLC=8、帧类型（标准/扩展）、邮箱和锁 */
}

void app_init(void)
{
    mwSHS_init(tx);
}

/* CAN 接收中断：标准帧传 StdId，扩展帧传 ExtId（≤0x7FF） */
void app_can_rx(uint8_t bus, uint32_t id, const uint8_t data[8], uint8_t dlc)
{
    if (id <= 0x7FF) mwSHS_recv(bus, (uint16_t)id, data, dlc);
}

/* 上电流程（第 ① 步：先等到心跳，即 mwSHS_get(0,1)->rx_count > 0） */
void motor_start(void)
{
    mwSHS_clear_errors(0, 1);
    mwSHS_set_mode(0, 1, MWSHS_CTRL_POS, MWSHS_INPUT_TRAP);
    mwSHS_set_limits(0, 1, 10.0f, 20.0f);      /* turns/s, A */
    mwSHS_set_traj_vel(0, 1, 5.0f);
    mwSHS_set_traj_accel(0, 1, 10.0f, 10.0f);
    mwSHS_set_traj_inertia(0, 1, 0.0f);
    mwSHS_set_state(0, 1, MWSHS_STATE_CLOSED_LOOP);
    /* 之后等 mwSHS_get(0,1)->state == MWSHS_STATE_CLOSED_LOOP */
}

/* 周期控制任务 */
void app_tick(void)
{
    const mwSHS_motor_t *m = mwSHS_get(0, 1);
    if (m && m->state == MWSHS_STATE_CLOSED_LOOP)
        mwSHS_pos(0, 1, 1.5f, 0, 0);
}

void motor_stop(void) { mwSHS_set_state(0, 1, MWSHS_STATE_IDLE); }
```

示例中的限值和轨迹参数只是占位值，要按实际机构整定。

---

## 12. 参考链接

- ODrive CAN Protocol（0.6.6 文档，**与本电调有不兼容之处**，见 [§1](#1-概述)）：https://docs.odriverobotics.com/v/0.6.6/can-protocol.html
- 厂商 servo_sdk：https://gitee.com/cyberbeast/servo_sdk
- 厂商 endpoint id JSON 下载：https://www.cyberbeast.cn/filedownload/837298
