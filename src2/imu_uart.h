/**
 ******************************************************************************
 * @file        imu_uart.h
 * @brief       ATOM-IMU 串口传输层（Zephyr）：USART10 DMA 接收
 ******************************************************************************
 */

#ifndef IMU_UART_H
#define IMU_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动 USART10 DMA 接收
 *
 * 依次完成：检查设备就绪 -> 清空 imu_data -> 清掉残余半帧 ->
 * 注册异步事件回调 -> 启动 DMA 接收。之后需在主循环中调用 imu_update()。
 *
 * USART10: RX = PE2, TX = PE3（引脚/波特率/时钟/DMA 通道均在设备树中配置）
 *
 * @return 0 成功，-ENODEV 设备未就绪，其他负值为注册回调/启动 DMA 失败的错误码
 */
int imu_uart_start(void);

/**
 * @brief 从环形缓冲取数据并组帧解析，结果写入 imu_data
 *
 * 内部调用 imu_encode()；一次调用会一直解析到剩余不足一帧为止，
 * 残帧留在暂存区等下次继续拼。
 *
 * 需要在主循环中周期调用，建议每 1 ms 一次（2 Mbaud 下约 3 帧/ms）。
 */
void imu_update(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_UART_H */
