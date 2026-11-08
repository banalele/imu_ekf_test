#include "zephyr/kernel.h"
#include "zephyr/logging/log.h"
#include <stdint.h>				// 整型类型
#include "rtt.h"
#include "imu_uart.h"
#include "imu_xrobot.h"
#include "bmi_ekf.hpp"
#include "imu.hpp"

using breeze::imu::ImuEkf;
using breeze::imu::imu_data_t;

LOG_MODULE_REGISTER(imu_algo_test, LOG_LEVEL_INF);
#define VOFA_PERIOD_MS 10

int main()
{
	vofa_rtt_init(); // 初始化 VOFA RTT 通道

	if (imu_uart_start() < 0)
	{
		LOG_ERR("ATOM-IMU 串口初始化失败");
	}

	int64_t vofa_tick = 0;
	static int64_t last_tick = 0;

	ImuEkf bmi_ekf;
	while (1)
	{
		/* 串口数据 -> 环形缓冲 -> 组帧解析 -> imu_data（每 1 ms 一次） */
		imu_update();

		const int64_t now = k_uptime_get();

		float dt = 0.001f; // 默认 1 ms
		if (last_tick != 0)
		{
			int64_t diff_ms = now - last_tick;
			if (diff_ms > 0 && diff_ms <= 10) // 合理范围：1~10 ms
			{
				dt = (float)diff_ms * 0.001f;
			}
		}
		last_tick = now;

		float acc[3] = {imu_data.accl_.x, imu_data.accl_.y, imu_data.accl_.z};
		float gyo[3] = {imu_data.gyro_.x, imu_data.gyro_.y, imu_data.gyro_.z};
		imu_data_t my_ekf = bmi_ekf.Update(acc,gyo,dt);

		/* RTT 发送降频到 VOFA_PERIOD_MS，避免日志链路拖慢主循环 */
		if ((now - vofa_tick) >= VOFA_PERIOD_MS)
		{
			vofa_tick = now;

			float vofa_data[] = {
				(float)k_uptime_get_32() / 1000.0f,																		   // 时间戳（秒）
				(float)((uint32_t)imu_data.time[0] | ((uint32_t)imu_data.time[1] << 8) |
						((uint32_t)imu_data.time[2] << 16) | ((uint32_t)imu_data.time[3] << 24)), // 微秒时间戳（低 32 位）
				(float)imu_data.eulr_.yaw,																				   // 偏航角（度）
				(float)imu_data.eulr_.pit,																				   // 俯仰角（度）
				(float)imu_data.eulr_.rol,	
				(float)my_ekf.yaw,
				(float)my_ekf.pitch,
				(float)my_ekf.roll,
			};
			// 将数据包写入 RTT 上行通道
			SEGGER_RTT_Write(RTT_CH_VOFA_1, vofa_data, sizeof(vofa_data));
			// 写入帧尾
			SEGGER_RTT_Write(RTT_CH_VOFA_1, vofa_tail, sizeof(vofa_tail));
		}

		k_sleep(K_MSEC(1));
	}
}
