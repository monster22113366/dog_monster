/*
 * 本文件属于 Bsp 硬件层，实现 MC02 板载 BMI088，依赖 main.h、spi.h 的 STM32 HAL 与 hspi2。
 * SPI2 Mode 3，PC0 为加速度计片选、PC3 为陀螺仪片选，均低有效；main.h 当前未生成片选宏。
 * 阻塞收发，超时 2 ms，不使用 DMA/中断，避开 H7 D-Cache 一致性问题。
 * 加速度计 800 Hz normal、±12 g；陀螺仪 1000 Hz / 116 Hz、±2000°/s。
 * 输出芯片系：+X 朝 USB/USART2-485，+Y 朝 CAN1/CAN2，+Z 朝元件面。
 * PB1 / TIM3_CH4 加热电阻直连 24 V，不得开启；本驱动不配置、不启动加热 PWM。
 */
#include "bsp_imu.h"
#include "main.h"
#include "spi.h"

#ifndef BSP_IMU_SPI_TIMEOUT_MS
#define BSP_IMU_SPI_TIMEOUT_MS 2u
#endif
#ifndef BSP_IMU_ACC_CS_PORT
#define BSP_IMU_ACC_CS_PORT GPIOC
#endif
#ifndef BSP_IMU_ACC_CS_PIN
#define BSP_IMU_ACC_CS_PIN GPIO_PIN_0
#endif
#ifndef BSP_IMU_GYRO_CS_PORT
#define BSP_IMU_GYRO_CS_PORT GPIOC
#endif
#ifndef BSP_IMU_GYRO_CS_PIN
#define BSP_IMU_GYRO_CS_PIN GPIO_PIN_3
#endif

static uint8_t s_ready;

/* imu_transfer：保持片选有效完成一次 SPI 事务
 * 参数：acc 为 1 选择加速度计，为 0 选择陀螺仪；tx、rx 为收发缓冲；len 为字节数
 * 返回：0 成功，-1 通信失败
 * 说明：无论成功或失败均释放片选；仅供初始化和主循环阻塞调用。
 */
static int imu_transfer(uint8_t acc, uint8_t *tx, uint8_t *rx, uint16_t len) {
    GPIO_TypeDef *port = acc ? BSP_IMU_ACC_CS_PORT : BSP_IMU_GYRO_CS_PORT;
    const uint16_t pin = acc ? BSP_IMU_ACC_CS_PIN : BSP_IMU_GYRO_CS_PIN;
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, len, BSP_IMU_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    return status == HAL_OK ? 0 : -1;
}

/* imu_read：读取连续寄存器
 * 参数：acc 选择传感器；reg 为首地址；out 接收数据；len 为数据长度，内部调用仅为 1 或 6
 * 返回：0 成功，-1 通信失败
 * 说明：加速度计地址后多收一个 dummy 字节并丢弃；陀螺仪无 dummy 字节。
 */
static int imu_read(uint8_t acc, uint8_t reg, uint8_t *out, uint8_t len) {
    uint8_t tx[8] = {0}, rx[8];
    const uint8_t offset = acc ? 2u : 1u;
    tx[0] = reg | 0x80u;
    if (imu_transfer(acc, tx, rx, (uint16_t)(len + offset)) != 0)
        return -1;
    for (uint8_t i = 0u; i < len; i++)
        out[i] = rx[i + offset];
    return 0;
}

/* imu_write：写入一个寄存器
 * 参数：acc 选择传感器；reg 为寄存器地址；value 为写入值
 * 返回：0 成功，-1 通信失败
 * 说明：两颗传感器的写事务均为地址加数据，共两个字节。
 */
static int imu_write(uint8_t acc, uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {(uint8_t)(reg & 0x7fu), value}, rx[2];
    return imu_transfer(acc, tx, rx, 2u);
}

/* bsp_imu_init：配置 BMI088 并逐项回读校验
 * 参数：无
 * 返回：0 成功；-1/-2 为 SPI 重配失败，-3~-15 为加速度计失败，-16~-21 为陀螺仪失败
 * 说明：各返回点对应独立步骤；任何失败均保持未就绪，不进入死循环。
 */
int bsp_imu_init(void) {
    uint8_t value;
    s_ready = 0u;
    HAL_GPIO_WritePin(BSP_IMU_ACC_CS_PORT, BSP_IMU_ACC_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BSP_IMU_GYRO_CS_PORT, BSP_IMU_GYRO_CS_PIN, GPIO_PIN_SET);
    if (hspi2.Init.BaudRatePrescaler != SPI_BAUDRATEPRESCALER_8) {
        /* CubeMX 当前 /2 = 37.5 MHz，超出 BMI088 的 10 MHz 上限；改 /8 = 9.375 MHz。
         * 建议日后在 CubeMX 中直接改为 /8，避免启动时重复初始化。 */
        hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
        if (HAL_SPI_DeInit(&hspi2) != HAL_OK)
            return -1;
        if (HAL_SPI_Init(&hspi2) != HAL_OK)
            return -2;
    }

    if (imu_read(1u, 0x00u, &value, 1u) != 0)
        return -3;
    if (imu_write(1u, 0x7eu, 0xb6u) != 0)
        return -4;
    HAL_Delay(50u);
    if (imu_read(1u, 0x00u, &value, 1u) != 0)
        return -5;
    if (imu_read(1u, 0x00u, &value, 1u) != 0)
        return -6;
    if (value != 0x1eu)
        return -7;
    if (imu_write(1u, 0x7du, 0x04u) != 0)
        return -8;
    HAL_Delay(50u);
    if (imu_write(1u, 0x7cu, 0x00u) != 0)
        return -9;
    if (imu_write(1u, 0x40u, 0xabu) != 0)
        return -10;
    if (imu_write(1u, 0x41u, 0x02u) != 0)
        return -11;
    if (imu_read(1u, 0x7du, &value, 1u) != 0 || value != 0x04u)
        return -12;
    if (imu_read(1u, 0x7cu, &value, 1u) != 0 || value != 0x00u)
        return -13;
    if (imu_read(1u, 0x40u, &value, 1u) != 0 || value != 0xabu)
        return -14;
    if (imu_read(1u, 0x41u, &value, 1u) != 0 || value != 0x02u)
        return -15;

    if (imu_write(0u, 0x14u, 0xb6u) != 0)
        return -16;
    HAL_Delay(50u);
    if (imu_read(0u, 0x00u, &value, 1u) != 0 || value != 0x0fu)
        return -17;
    if (imu_write(0u, 0x0fu, 0x00u) != 0)
        return -18;
    if (imu_write(0u, 0x10u, 0x02u) != 0)
        return -19;
    if (imu_read(0u, 0x0fu, &value, 1u) != 0 || value != 0x00u)
        return -20;
    /* GYRO_BANDWIDTH bit7 为只读位，恒读为 1，校验时屏蔽。 */
    if (imu_read(0u, 0x10u, &value, 1u) != 0 || (value & 0x7fu) != 0x02u)
        return -21;
    s_ready = 1u;
    return 0;
}

/* bsp_imu_read：读取两颗传感器的最新值并换算单位
 * 参数：out 接收芯片坐标系样本
 * 返回：0 成功，1 未初始化，-1 空指针，-2 加速度计通信失败，-3 陀螺仪通信失败
 * 说明：小端有符号 16 位；读取全部成功后才写 out，不轮询 data-ready、不延时。
 */
int bsp_imu_read(bsp_imu_sample_t *out) {
    uint8_t acc[6], gyro[6];
    if (!s_ready)
        return 1;
    if (out == 0)
        return -1;
    if (imu_read(1u, 0x12u, acc, 6u) != 0)
        return -2;
    if (imu_read(0u, 0x02u, gyro, 6u) != 0)
        return -3;
    for (uint8_t i = 0u; i < 3u; i++) {
        const int16_t a = (int16_t)((uint16_t)acc[2u * i] | ((uint16_t)acc[2u * i + 1u] << 8));
        const int16_t w = (int16_t)((uint16_t)gyro[2u * i] | ((uint16_t)gyro[2u * i + 1u] << 8));
        out->acc[i] = (float)a / 32768.0f * 12.0f * 9.81f;
        out->gyro[i] = (float)w / 32768.0f * 2000.0f * (3.14159265358979323846f / 180.0f);
    }
    return 0;
}
