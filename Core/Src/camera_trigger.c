#include "camera_trigger.h"

#include <stddef.h>

#include "imu_stream.h"
#include "imu_task.h"

#define CAMERA_TRIGGER_EVENT_DEPTH 4U

static TIM_HandleTypeDef *s_timer;
static volatile uint32_t s_sequence;
static volatile uint32_t s_dropped;
static volatile uint8_t s_head;
static volatile uint8_t s_tail;
static ImuStreamSyncEvent s_events[CAMERA_TRIGGER_EVENT_DEPTH];

void CameraTrigger_Init(TIM_HandleTypeDef *htim) {
  s_timer = htim;
  s_sequence = 0U;
  s_dropped = 0U;
  s_head = 0U;
  s_tail = 0U;

  if (s_timer == NULL) {
    return;
  }
  (void)HAL_TIM_PWM_Start(s_timer, CAMERA_TRIGGER_CHANNEL);
  __HAL_TIM_ENABLE_IT(s_timer, TIM_IT_UPDATE);
}

void CameraTrigger_OnPeriodElapsed(TIM_HandleTypeDef *htim) {
  uint8_t next;
  ImuStreamSyncEvent *event;

  if ((s_timer == NULL) || (htim != s_timer)) {
    return;
  }

  next = (uint8_t)((s_head + 1U) % CAMERA_TRIGGER_EVENT_DEPTH);
  if (next == s_tail) {
    ++s_dropped;
    ++s_sequence;
    return;
  }

  event = &s_events[s_head];
  event->seq = s_sequence++;
  event->trigger_ns = ImuTask_NowNs();
  event->strobe_ns = 0U;
  event->flags = 0U;
  __DMB();
  s_head = next;
}

void CameraTrigger_Poll(void) {
  ImuStreamSyncEvent event;

  while (s_tail != s_head) {
    __DMB();
    event = s_events[s_tail];
    s_tail = (uint8_t)((s_tail + 1U) % CAMERA_TRIGGER_EVENT_DEPTH);
    (void)ImuStream_SendSync(&event);
  }
}

uint32_t CameraTrigger_GetSequence(void) {
  return s_sequence;
}

uint32_t CameraTrigger_GetDroppedEvents(void) {
  return s_dropped;
}
