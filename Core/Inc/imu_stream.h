#ifndef IMU_STREAM_H
#define IMU_STREAM_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

/*
 * Binary framing for the IMU stream. Replaces the former JustFloat/VOFA+
 * encoding that this project started with, which was designed for plotting and
 * could only carry float32.
 *
 * Frame layout (31 bytes, fixed):
 *
 *   offset  size  field
 *   0       2     sync   0xAA 0x55
 *   2       1     version (IMU_STREAM_VERSION)
 *   3       1     length  (payload length in bytes)
 *   4       25    payload (see ImuStreamSample)
 *   29      2     CRC-16/CCITT-FALSE over bytes [2, 29)
 *
 * Payload layout:
 *   offset  size  field
 *   0       8     uint64 timestamp_ns   (since boot, see imu_task.c)
 *   8       6     3 x int16 gyro  raw LSB, X/Y/Z
 *   14      6     3 x int16 accel raw LSB, X/Y/Z
 *   20      2     int16 temperature code (11-bit two's complement)
 *   22      1     uint8 gyro range register value (0..4)
 *   23      1     uint8 accel range register value (0..3)
 *   24      1     uint8 flags
 *
 * Raw 16-bit values are sent instead of converted floats: the measurement path
 * stays integer end to end, and the ranges travel with every sample so the host
 * derives the scale factor itself (one source of truth for the units).
 *
 * A fixed length plus the sync word and a CRC makes the stream self
 * synchronising: the host only has to scan for the sync word and reject frames
 * whose CRC does not match.
 */

#define IMU_STREAM_SYNC_0 0xAAU
#define IMU_STREAM_SYNC_1 0x55U
#define IMU_STREAM_VERSION 0x01U
#define IMU_STREAM_VERSION_SYNC 0x02U

#define IMU_STREAM_PAYLOAD_BYTES 25U
#define IMU_STREAM_HEADER_BYTES 4U
#define IMU_STREAM_CRC_BYTES 2U
#define IMU_STREAM_FRAME_BYTES \
  (IMU_STREAM_HEADER_BYTES + IMU_STREAM_PAYLOAD_BYTES + IMU_STREAM_CRC_BYTES)

/* Frame slots in the TX ring. */
#define IMU_STREAM_QUEUE_DEPTH 4U

/*
 * Slots that may be occupied at once. The slot owned by the DMA engine stays
 * outside the producer/consumer window, so one slot is always left unused;
 * three frames still absorb several milliseconds of host stall.
 */
#define IMU_STREAM_MAX_OUTSTANDING (IMU_STREAM_QUEUE_DEPTH - 1U)

/* sample.flags bits */
#define IMU_STREAM_FLAG_SAMPLE_LOST 0x01U /* main loop missed a sensor sample */
#define IMU_STREAM_FLAG_READ_ERROR 0x02U  /* an SPI read failed since last frame */

typedef struct {
  uint64_t timestamp_ns;  /* capture time of the data-ready edge */
  int16_t gyro_raw[3];    /* sensor LSB */
  int16_t accel_raw[3];   /* sensor LSB */
  int16_t temperature_code;
  uint8_t gyro_range;     /* GYRO_RANGE register value */
  uint8_t accel_range;    /* ACC_RANGE register value */
  uint8_t flags;
} ImuStreamSample;

typedef struct {
  uint32_t seq;
  uint64_t trigger_ns;
  uint64_t strobe_ns;
  uint8_t flags;
} ImuStreamSyncEvent;

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor. */
uint16_t ImuStream_Crc16(const uint8_t *data, uint16_t length);

/* huart must be configured for 8N1 TX with a linked TX DMA channel. */
void ImuStream_Init(UART_HandleTypeDef *huart);

/*
 * Queues one frame. Returns false when the queue is full, which can only happen
 * if the caller falls several frames behind.
 */
bool ImuStream_Send(const ImuStreamSample *sample);

bool ImuStream_SendSync(const ImuStreamSyncEvent *event);

/* Kicks the next queued frame out when the UART is idle. Main loop only. */
void ImuStream_Poll(void);

/* Call from HAL_UART_TxCpltCallback(). */
void ImuStream_OnTxComplete(UART_HandleTypeDef *huart);

#endif /* IMU_STREAM_H */
