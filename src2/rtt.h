#ifndef RTT_H
#define RTT_H
#include <stdint.h>
#include <SEGGER_RTT.h>

#define RTT_CH_LOG 0	// 通道 0 RTT 日志输出通道
#define RTT_CH_VOFA_1 1 // 通道 1
#define RTT_CH_VOFA_2 2 // 通道 2

#ifdef __cplusplus
extern "C" {
#endif

// VOFA+ JustFloat 模式帧尾（4 字节），发送完数据包后追加
extern const uint8_t vofa_tail[4];

void vofa_rtt_init(void);

#ifdef __cplusplus
}
#endif

#endif /* RTT_H */