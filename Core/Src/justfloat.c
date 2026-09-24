#include "justfloat.h"

#include <string.h>

/* Little-endian image of +Infinity (0x7F800000): the JustFloat frame tail. */
static const uint8_t s_frame_tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};

/* The producer/consumer indices live in the main loop; only s_in_flight is
 * written by the TX-complete interrupt. */
static UART_HandleTypeDef *s_uart;
static uint8_t s_queue[JUSTFLOAT_QUEUE_DEPTH][JUSTFLOAT_FRAME_BYTES];
static uint16_t s_head;             /* next slot the producer writes */
static uint16_t s_tail;             /* next slot the consumer sends */
static uint16_t s_queued;           /* frames waiting in the ring */
static volatile uint8_t s_in_flight; /* a DMA transfer owns the slot s_tail - 1 */

void JustFloat_Init(UART_HandleTypeDef *huart) {
  s_uart = huart;
  s_head = 0U;
  s_tail = 0U;
  s_queued = 0U;
  s_in_flight = 0U;
}

bool JustFloat_Send(const float *channels) {
  uint8_t *frame;

  if ((s_uart == NULL) || (channels == NULL) ||
      ((uint16_t)(s_queued + s_in_flight) >= JUSTFLOAT_MAX_OUTSTANDING)) {
    return false;
  }

  frame = s_queue[s_head];
  for (uint16_t i = 0U; i < JUSTFLOAT_CHANNEL_COUNT; ++i) {
    /*
     * Cortex-M4 is little-endian, so the in-memory image of a float already is
     * the little-endian IEEE-754 encoding JustFloat expects.
     */
    memcpy(&frame[i * 4U], &channels[i], sizeof(float));
  }
  memcpy(&frame[JUSTFLOAT_FRAME_BYTES - 4U], s_frame_tail,
         sizeof(s_frame_tail));

  s_head = (uint16_t)((s_head + 1U) % JUSTFLOAT_QUEUE_DEPTH);
  ++s_queued;
  return true;
}

void JustFloat_Poll(void) {
  uint8_t *frame;

  if ((s_uart == NULL) || (s_queued == 0U) || (s_in_flight != 0U)) {
    return;
  }
  if (HAL_UART_GetState(s_uart) != HAL_UART_STATE_READY) {
    return;
  }

  /* Take the oldest frame out of the queue before handing it to the DMA. */
  frame = s_queue[s_tail];
  s_tail = (uint16_t)((s_tail + 1U) % JUSTFLOAT_QUEUE_DEPTH);
  --s_queued;
  s_in_flight = 1U;

  if (HAL_UART_Transmit_DMA(s_uart, frame, JUSTFLOAT_FRAME_BYTES) != HAL_OK) {
    s_in_flight = 0U;
  }
}

void JustFloat_OnTxComplete(UART_HandleTypeDef *huart) {
  if (huart == s_uart) {
    s_in_flight = 0U;
  }
}
