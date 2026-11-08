#include "rtt.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rtt, LOG_LEVEL_INF);
static uint8_t vofa_buf_1[1024]; // 用于通道 1
static uint8_t vofa_buf_2[1024]; // 用于通道 2
// VOFA+ JustFloat模式 帧尾
const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7F};
void vofa_rtt_init(void)
{
    // 上行通道
	SEGGER_RTT_ConfigUpBuffer(
		RTT_CH_VOFA_1,				  // 通道索引
		"vofa_channel_1",			  // 通道名称（仅供调试器显示）
		vofa_buf_1,					  // 缓冲区地址
		sizeof(vofa_buf_1),			  // 缓冲区大小
		SEGGER_RTT_MODE_NO_BLOCK_SKIP // 无阻塞模式：空间不足时丢弃新数据，不等待
	);
	SEGGER_RTT_ConfigUpBuffer(
		RTT_CH_VOFA_2,
		"vofa_channel_2",
		vofa_buf_2,
		sizeof(vofa_buf_2),
		SEGGER_RTT_MODE_NO_BLOCK_SKIP);

	LOG_INF("VOFA RTT channel configured\r\n");
}
