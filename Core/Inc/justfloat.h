#ifndef JUSTFLOAT_H
#define JUSTFLOAT_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

/*
 * JustFloat is the VOFA+ "raw float stream" frame format:
 *
 *   frame = [float32 ch0][float32 ch1] ... [float32 chN-1][tail]
 *   tail  = 0x00 0x00 0x80 0x7F
 *
 * Every float is little-endian IEEE-754 single precision. The four tail bytes
 * are the little-endian image of 0x7F800000, that is +Infinity, which VOFA+
 * uses as the frame delimiter. A frame is therefore N * 4 + 4 bytes.
 *
 * Reference: https://www.vofa.plus/docs/learning/dataengines/justfloat
 *
 * Frames are handed to the UART with DMA through a small queue so the sampling
 * loop never blocks on the serial line.
 */

/* Channels per frame; VOFA+ must be set to the same count. */
#define JUSTFLOAT_CHANNEL_COUNT 9U

/* Frame slots in the TX ring. */
#define JUSTFLOAT_QUEUE_DEPTH 4U

/*
 * Slots that may be occupied at once. The slot owned by the DMA engine stays
 * outside the producer/consumer window, so one slot is always left unused;
 * three frames still absorb 3 ms of jitter at 1 kHz.
 */
#define JUSTFLOAT_MAX_OUTSTANDING (JUSTFLOAT_QUEUE_DEPTH - 1U)

#define JUSTFLOAT_FRAME_BYTES (JUSTFLOAT_CHANNEL_COUNT * 4U + 4U)

/* huart must be configured for 8N1 TX with a linked TX DMA channel. */
void JustFloat_Init(UART_HandleTypeDef *huart);

/*
 * Queues one frame of JUSTFLOAT_CHANNEL_COUNT values. Returns false when the
 * queue is full, which can only happen if the caller falls three frames behind.
 */
bool JustFloat_Send(const float *channels);

/* Kicks the next queued frame out when the UART is idle. Main loop only. */
void JustFloat_Poll(void);

/* Call from HAL_UART_TxCpltCallback(). */
void JustFloat_OnTxComplete(UART_HandleTypeDef *huart);

#endif /* JUSTFLOAT_H */
