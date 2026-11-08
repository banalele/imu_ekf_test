/**
 ******************************************************************************
 * @file        imu_xrobot.h
 * @brief       ATOM-IMU 成品陀螺仪 UART 通信协议层（Zephyr 移植版）
 *
 * 参照 HAL 版 imu_xrobot.h / imu_xrobot.c 移植，差别只有传输层：
 *   HAL 版  : USART1 + DMA 循环 + 空闲中断，在中断里解析
 *   Zephyr 版: USART10（RX=PE2, TX=PE3, 2000000 8N1）DMA 接收 + 环形缓冲，
 *              主循环每 1 ms 调用 imu_update() -> imu_encode() 解析
 *
 * 数据仍然放在全局变量 imu_data 中，任意位置 extern 后直接读取：
 *
 * // 欧拉角（度，驱动内部已从弧度转换）
 * float roll  = imu_data.eulr_.rol;
 * float pitch = imu_data.eulr_.pit;
 * float yaw   = imu_data.eulr_.yaw;
 *
 * // 角速度（rad/s）
 * float gyro_x = imu_data.gyro_.x;
 *
 * // 加速度（m/s^2）
 * float acc_x = imu_data.accl_.x;
 *
 * // 四元数
 * float q0 = imu_data.quat_.q0;
 *
 * ==== 数据帧格式（共 64 字节，小端）====
 *
 * | 偏移 | 字段   | 大小    | 说明                       |
 * |------|--------|---------|----------------------------|
 * | 0    | prefix | 1 字节  | 帧头，固定 0xA5            |
 * | 1    | time   | 5 字节  | 微秒时间戳 (40-bit)        |
 * | 6    | sync   | 5 字节  | 同步时间戳 (40-bit)        |
 * | 11   | quat_  | 16 字节 | 四元数 (4 * float)         |
 * | 27   | gyro_  | 12 字节 | 角速度 (3 * float, rad/s)  |
 * | 39   | accl_  | 12 字节 | 加速度 (3 * float, m/s^2)  |
 * | 51   | eulr_  | 12 字节 | 欧拉角 (3 * float, 弧度)   |
 * | 63   | crc8   | 1 字节  | CRC8 校验 (poly 0x31, 初值 0xFF) |
 ******************************************************************************
 */

#ifndef IMU_XROBOT_H
#define IMU_XROBOT_H

#include <stdint.h>

#define IMU_FRAME_SIZE 64
#define IMU_FRAME_HEAD 0xA5
#define IMU_CRC_INIT 0xFF

/* 弧度转度 (180/π) */
#define RAD_TO_DEG 57.29578f

/* Exported types ------------------------------------------------------------*/

/** @brief 三维向量 */
typedef struct __attribute__((packed))
{
	float x;
	float y;
	float z;
} Vector3;

/** @brief 四元数 */
typedef struct __attribute__((packed))
{
	float q0;
	float q1;
	float q2;
	float q3;
} Quaternion;

/** @brief 欧拉角（字段顺序与线上字节序一致：roll, pitch, yaw） */
typedef struct __attribute__((packed))
{
	float rol; // 横滚角 (度)
	float pit; // 俯仰角 (度)
	float yaw; // 偏航角 (度)
} EulerAngles;

/** @brief 完整 IMU 数据帧结构 */
typedef struct __attribute__((packed))
{
	uint8_t prefix;	  // 帧头，固定 0xA5
	uint8_t time[5];  // 微秒时间戳 (40-bit)
	uint8_t sync[5];  // 同步时间戳 (40-bit)
	Quaternion quat_; // 四元数
	Vector3 gyro_;	  // 陀螺仪角速度 (rad/s)
	Vector3 accl_;	  // 加速度 (m/s^2)
	EulerAngles eulr_; // 欧拉角 (度，解析时已由弧度转换)
	uint8_t crc8;	  // CRC8 校验
} ImuUartData;

_Static_assert(sizeof(ImuUartData) == IMU_FRAME_SIZE,
			   "ImuUartData 必须与线上 64 字节帧一致");

/* Exported variables --------------------------------------------------------*/

/** @brief 最新一帧 IMU 数据（解析成功才会刷新） */
extern ImuUartData imu_data;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief CRC8 计算（查表法）
 *
 * 对应 IMU 帧尾校验：多项式 0x31（反射形式），初始值 0xFF。
 *
 * @param data 待校验数据
 * @param len  字节数（不包括 CRC 本身）
 * @return 计算出的 CRC8 值
 */
uint8_t imu_crc8_calc(const uint8_t *data, uint16_t len);

/**
 * @brief 清空 imu_data 与帧统计计数
 *
 * 在开始接收前调用（imu_uart_start() 内已调用）。
 */
void imu_xrobot_init(void);

/**
 * @brief 从一段原始字节中找帧头、校验 CRC 并更新 imu_data
 *
 * 处理流程：
 *   1. 跳过帧头 0xA5 之前的杂字节；
 *   2. 凑满 IMU_FRAME_SIZE（64 字节）后做 CRC8 校验；
 *   3. 校验通过：整帧拷入 imu_data，并把欧拉角由弧度转为度；
 *      校验失败：只增加 crc 错误计数，imu_data 保持上一帧不变；
 *   4. 无论成败都前进 64 字节，继续解析后面的帧。
 *
 * @param rx_buf 数据缓冲区（不必按帧对齐）
 * @param rx_len 数据长度
 * @return 已消费的字节数；从帧头开始不足一帧时返回帧头位置，由调用者保留
 */
uint16_t imu_encode(const uint8_t *rx_buf, uint16_t rx_len);

#endif /* IMU_XROBOT_H */
