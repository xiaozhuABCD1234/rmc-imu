#include "imu_stream.h"

#include <string.h>

/* The producer/consumer indices live in the main loop; only s_in_flight is
 * written by the TX-complete interrupt. */
#define IMU_STREAM_SYNC_QUEUE_DEPTH 4U

static UART_HandleTypeDef *s_uart;
static uint8_t s_queue[IMU_STREAM_QUEUE_DEPTH][IMU_STREAM_FRAME_BYTES];
static uint16_t s_head;              /* next slot the producer writes */
static uint16_t s_tail;              /* next slot the consumer sends */
static uint16_t s_queued;            /* frames waiting in the ring */
static uint8_t s_sync_queue[IMU_STREAM_SYNC_QUEUE_DEPTH][IMU_STREAM_FRAME_BYTES];
static uint16_t s_sync_head;
static uint16_t s_sync_tail;
static uint16_t s_sync_queued;
static volatile uint8_t s_in_flight; /* a DMA transfer owns the slot s_tail - 1 */

uint16_t ImuStream_Crc16(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFFU;

  for (uint16_t i = 0U; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      crc = ((crc & 0x8000U) != 0U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                    : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static void ImuStream_PutU16(uint8_t *destination, uint16_t value) {
  destination[0] = (uint8_t)(value & 0xFFU);
  destination[1] = (uint8_t)(value >> 8);
}

static void ImuStream_PutU32(uint8_t *destination, uint32_t value) {
  for (uint8_t i = 0U; i < 4U; ++i) {
    destination[i] = (uint8_t)((value >> (8U * i)) & 0xFFU);
  }
}

static void ImuStream_PutU64(uint8_t *destination, uint64_t value) {
  for (uint8_t i = 0U; i < 8U; ++i) {
    destination[i] = (uint8_t)((value >> (8U * i)) & 0xFFU);
  }
}

void ImuStream_Init(UART_HandleTypeDef *huart) {
  s_uart = huart;
  s_head = 0U;
  s_tail = 0U;
  s_queued = 0U;
  s_sync_head = 0U;
  s_sync_tail = 0U;
  s_sync_queued = 0U;
  s_in_flight = 0U;
}

bool ImuStream_Send(const ImuStreamSample *sample) {
  uint8_t *frame;
  uint8_t *payload;
  uint8_t axis;
  uint16_t crc;

  if ((s_uart == NULL) || (sample == NULL) ||
      ((uint16_t)(s_queued + s_in_flight) >= IMU_STREAM_MAX_OUTSTANDING)) {
    return false;
  }

  frame = s_queue[s_head];
  payload = &frame[IMU_STREAM_HEADER_BYTES];

  frame[0] = IMU_STREAM_SYNC_0;
  frame[1] = IMU_STREAM_SYNC_1;
  frame[2] = IMU_STREAM_VERSION;
  frame[3] = IMU_STREAM_PAYLOAD_BYTES;

  ImuStream_PutU64(&payload[0], sample->timestamp_ns);
  for (axis = 0U; axis < 3U; ++axis) {
    ImuStream_PutU16(&payload[8U + axis * 2U],
                     (uint16_t)sample->gyro_raw[axis]);
    ImuStream_PutU16(&payload[14U + axis * 2U],
                     (uint16_t)sample->accel_raw[axis]);
  }
  ImuStream_PutU16(&payload[20], (uint16_t)sample->temperature_code);
  payload[22] = sample->gyro_range;
  payload[23] = sample->accel_range;
  payload[24] = sample->flags;

  /* CRC covers version, length and payload, but not the sync word. */
  crc = ImuStream_Crc16(&frame[2], (uint16_t)(2U + IMU_STREAM_PAYLOAD_BYTES));
  ImuStream_PutU16(&frame[IMU_STREAM_HEADER_BYTES + IMU_STREAM_PAYLOAD_BYTES],
                   crc);

  s_head = (uint16_t)((s_head + 1U) % IMU_STREAM_QUEUE_DEPTH);
  ++s_queued;
  return true;
}

bool ImuStream_SendSync(const ImuStreamSyncEvent *event) {
  uint8_t *frame;
  uint8_t *payload;
  uint16_t crc;

  if ((event == NULL) ||
      (s_sync_queued >= IMU_STREAM_SYNC_QUEUE_DEPTH)) {
    return false;
  }

  frame = s_sync_queue[s_sync_head];
  payload = &frame[IMU_STREAM_HEADER_BYTES];

  frame[0] = IMU_STREAM_SYNC_0;
  frame[1] = IMU_STREAM_SYNC_1;
  frame[2] = IMU_STREAM_VERSION_SYNC;
  frame[3] = IMU_STREAM_PAYLOAD_BYTES;

  ImuStream_PutU32(&payload[0], event->seq);
  ImuStream_PutU64(&payload[4], event->trigger_ns);
  ImuStream_PutU64(&payload[12], event->strobe_ns);
  payload[20] = event->flags;
  payload[21] = 0U;
  payload[22] = 0U;
  payload[23] = 0U;
  payload[24] = 0U;

  crc = ImuStream_Crc16(&frame[2], (uint16_t)(2U + IMU_STREAM_PAYLOAD_BYTES));
  ImuStream_PutU16(&frame[IMU_STREAM_HEADER_BYTES + IMU_STREAM_PAYLOAD_BYTES],
                   crc);

  s_sync_head = (uint16_t)((s_sync_head + 1U) % IMU_STREAM_SYNC_QUEUE_DEPTH);
  ++s_sync_queued;
  return true;
}

void ImuStream_Poll(void) {
  uint8_t *frame;

  if ((s_uart == NULL) || (s_in_flight != 0U)) {
    return;
  }
  if (HAL_UART_GetState(s_uart) != HAL_UART_STATE_READY) {
    return;
  }

  if (s_sync_queued > 0U) {
    frame = s_sync_queue[s_sync_tail];
    s_sync_tail =
        (uint16_t)((s_sync_tail + 1U) % IMU_STREAM_SYNC_QUEUE_DEPTH);
    --s_sync_queued;
    s_in_flight = 1U;

    if (HAL_UART_Transmit_DMA(s_uart, frame, IMU_STREAM_FRAME_BYTES) !=
        HAL_OK) {
      s_in_flight = 0U;
    }
    return;
  }

  if (s_queued == 0U) {
    return;
  }

  /* Take the oldest frame out of the queue before handing it to the DMA. */
  frame = s_queue[s_tail];
  s_tail = (uint16_t)((s_tail + 1U) % IMU_STREAM_QUEUE_DEPTH);
  --s_queued;
  s_in_flight = 1U;

  if (HAL_UART_Transmit_DMA(s_uart, frame, IMU_STREAM_FRAME_BYTES) != HAL_OK) {
    s_in_flight = 0U;
  }
}

void ImuStream_OnTxComplete(UART_HandleTypeDef *huart) {
  if (huart == s_uart) {
    s_in_flight = 0U;
  }
}
