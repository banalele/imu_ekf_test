/**
 ******************************************************************************
 * @file        imu_uart.c
 * @brief       ATOM-IMU 串口传输层（Zephyr）：USART10 DMA 接收
 *
 * 对应 HAL 版 drv_uart.c 中 USART1 + DMA 循环 + 空闲中断：
 *   DMA 直接把字节搬进 nocache 缓冲，收满一块或总线空闲时回调，
 *   回调里只把数据拷进环形缓冲；主循环每 1 ms 调用 imu_update() 解析。
 *
 * 调用关系：USART10 DMA -> 环形缓冲 -> imu_update() -> imu_encode()
 *
 * 设备树：damiao_mc02 的 usart10（RX=PE2, TX=PE3, 2000000 8N1, 已开 FIFO）
 ******************************************************************************
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#include "imu_uart.h"
#include "imu_xrobot.h"

LOG_MODULE_REGISTER(imu_uart, LOG_LEVEL_INF);

#define IMU_UART_NODE DT_NODELABEL(usart10)

/* 环形缓冲：2000000 baud 下约 20 ms 的数据量（DMA 回调写、线程读） */
#define IMU_RX_RING_SIZE 4096
/* 解析暂存区，对应 HAL 版的 USART1_RX_BUF_LEN */
#define IMU_RX_STAGE_SIZE 128
/* DMA 乒乓缓冲大小：256 B 约 1.28 ms 数据 */
#define IMU_DMA_BUF_SIZE 256

static const struct device *const imu_dev = DEVICE_DT_GET(IMU_UART_NODE);

RING_BUF_DECLARE(imu_ring_buffer, IMU_RX_RING_SIZE);

/* DMA 目标缓冲必须在 nocache 段，否则 D-Cache 会读到旧数据 */
static uint8_t dma_buf_a[IMU_DMA_BUF_SIZE] __nocache __aligned(32);
static uint8_t dma_buf_b[IMU_DMA_BUF_SIZE] __nocache __aligned(32);
/* 空闲的备用缓冲，驱动请求时交出去；NULL 表示暂无 */
static uint8_t *dma_spare = dma_buf_b;

/* 未解析完的尾部数据暂存（跨 imu_update() 调用保留） */
static uint8_t stage_buf[IMU_RX_STAGE_SIZE];
static size_t stage_len;

/**
 * @brief USART10 接收回调（DMA 异步事件，运行在中断上下文）
 *
 * RX_RDY          ：DMA 已收到数据，拷进环形缓冲
 * RX_BUF_REQUEST  ：驱动要下一块缓冲，交出 dma_spare
 * RX_BUF_RELEASED ：驱动用完一块，回收成 dma_spare
 * RX_DISABLED     ：出错/被停，重新开启接收
 */
static void USART10_rxDataHandler(const struct device *dev,
				  struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	switch (evt->type)
	{
	case UART_RX_RDY:
		k_msgq_put()
		(void) ring_buf_put(&imu_ring_buffer,
									  &evt->data.rx.buf[evt->data.rx.offset],
									  (uint32_t)evt->data.rx.len);
		break;

	case UART_RX_BUF_REQUEST:
		if (dma_spare != NULL)
		{
			uint8_t *buf = dma_spare;

			dma_spare = NULL;
			(void)uart_rx_buf_rsp(dev, buf, IMU_DMA_BUF_SIZE);
		}
		break;

	case UART_RX_BUF_RELEASED:
		dma_spare = evt->data.rx_buf.buf;
		break;

	case UART_RX_DISABLED:
		/* 重新开始：a 交给驱动，b 留作备用 */
		dma_spare = dma_buf_b;
		(void)uart_rx_enable(dev, dma_buf_a, IMU_DMA_BUF_SIZE, 0);
		break;

	default:
		break;
	}
}

int imu_uart_start(void)
{
	int ret;

	if (!device_is_ready(imu_dev))
	{
		LOG_ERR("USART10 未就绪，请检查设备树 DT_NODELABEL(usart10)");
		return -ENODEV;
	}

	imu_xrobot_init(); /* 清空 imu_data */

	stage_len = 0; /* 丢掉可能残留的半帧 */

	ret = uart_callback_set(imu_dev, USART10_rxDataHandler, NULL);
	if (ret < 0)
	{
		LOG_ERR("注册 UART 异步回调失败 (%d)，需要 CONFIG_UART_ASYNC_API=y", ret);
		return ret;
	}

	/* 乒乓缓冲：a 交给驱动，b 留作备用 */
	dma_spare = dma_buf_b;

	/* timeout = 0：用总线空闲(IDLE)检测分包；DMA 收满一整块也会回调 */
	ret = uart_rx_enable(imu_dev, dma_buf_a, IMU_DMA_BUF_SIZE, 0);
	if (ret < 0)
	{
		LOG_ERR("启动 USART10 DMA 接收失败 (%d)，检查 dmas / DMAMUX / nocache 配置", ret);
		return ret;
	}

	LOG_INF("ATOM-IMU DMA 接收已启动：%s", imu_dev->name);

	return 0;
}

void imu_update(void)
{
	uint16_t used;

	/* 一次调用可能积压多帧（ring 比暂存区大），循环取到只剩半帧为止 */
	for (;;)
	{
		/* 1. 环形缓冲 -> 暂存区尾部（stage_len 同时充当写指针） */
		stage_len += ring_buf_get(&imu_ring_buffer, &stage_buf[stage_len],
								  sizeof(stage_buf) - stage_len);

		/* 2. 解析，返回真正吃掉多少字节 */
		used = imu_encode(stage_buf, (uint16_t)stage_len);

		if (used == 0)
		{
			break; /* 凑不满一帧，等下一批数据 */
		}

		/* 3. 丢弃已解析部分，把残余的半帧挪到 buf 开头 */
		stage_len -= used;
		memmove(stage_buf, &stage_buf[used], stage_len);
	}
}
